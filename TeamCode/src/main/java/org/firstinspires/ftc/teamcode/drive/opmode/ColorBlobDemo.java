package org.firstinspires.ftc.teamcode.drive.opmode;

import android.graphics.drawable.GradientDrawable;
import android.transition.Slide;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.calib3d.Calib3d;

/**
 * ColorBlobDemo - A demo OpMode that shows how to use the ColorBlobDetector
 * to detect colored objects and calculate their real-world distance and orientation
 * from the robot's center.
 * 
 * This OpMode demonstrates:
 * 1. How to initialize the ColorBlobDetector with camera position parameters
 * 2. How to detect colored objects and get their real-world measurements
 * 3. How to switch between different target colors during operation
 * 4. How to use the measurements for robot guidance and navigation
 */
@Config
@TeleOp(name = "Color Blob Demo", group = "Demo")
public class ColorBlobDemo extends LinearOpMode {
    public static double fx = 1430;
    public static double fy = 1430;
    public final double cx = 320;
    public final double cy = 240;
    private static double k1 = -0.0151731;
    private double k2 = 1.19665;
    private double k3 = -4.01517;
    private double p1 = -0.00113377;
    private double p2 = 0.0015228;

    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
    public double[] data = {
            0, 0, fx,
            0, 1, 0,
            0, 2, cx,
            1, 0, 0,
            1, 1, fy,
            1, 2, cy,
            2, 0, 0,
            2, 1, 0,
            2, 2, 1
    };

    public void setCameraMatrix(Mat cameraMatrix) {
        this.cameraMatrix = cameraMatrix;
    }

    public Servo ClawServo;
    public Servo WristServo;
    public Servo ShoulderServo;
    private Servo turret;
    private Servo pickUpPivot;
    public static double step = 22.5;

    public static long wait = 2000;

    private double changeVerticalDepth(double depth){
        return 0.129 * depth + 6.07;
    }
    private double changeHorizontalDepth(double depth){
        return -0.316*depth + 1.52;
    }
    public void MoveSlideandTurret(double YOffset, double XOffset) {
        double NewY = Math.sqrt(ARMLENGTH*ARMLENGTH - XOffset * XOffset);
        double SlideMove = YOffset - NewY;
        turretMovementAngle = Math.toDegrees(Math.atan(XOffset/NewY));
        telemetry.addData("SLIDE MOTOR: ", "MOVING");
        SlideMotor.setTargetPosition((int) (SlideMove * TICKSPERINCHES));
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setVelocity(velocity);
        telemetry.addData("Moving slide Inches: ", (int) (SlideMove * TICKSPERINCHES));
//        try {
//            Thread.sleep(100);
//        }
//        catch (InterruptedException e){
//            telemetry.addData("Error: ", e);
//        }
        if (!SlideMotor.isBusy()) {
            telemetry.addData("SLIDE MOTOR: ", "STOPPED");
            turret.setPosition(0.5 + (turretMovementAngle/300));
            moveShoulderUp = false; // Give the command to move the shoulder down
            telemetry.addData("Turret Angle: ", turretMovementAngle);
            telemetry.addData("Turret Position: ", 0.5 - ((turretMovementAngle)/300));
            telemetry.addData("Current Servo Pos: ", turret.getPosition());
        }
        telemetry.update();
    }
    public void MoveShoulder(){
        telemetry.addData("MOVING SHOULDER:", "DOWN");
        ShoulderServo.setPosition(0.47); //pickup position
    }
    private void rotateWristServo(double angleDeg){
        telemetry.addData("MOVING WRIST SERVO: ", "MOVING");
        angleDeg -= turretMovementAngle;
        telemetry.addData("ANGLE DEGREES: ", angleDeg);
        telemetry.addData("TURRET MOVEMENT ANGLE: ", turretMovementAngle);
//        if (angleDeg >= (45 + step) || angleDeg <= (-45 - step)){
//            WristServo.setPosition(0.5);
//            telemetry.addData("Going to POS", "VERTICAL");
//        } else if (angleDeg < (90 - step) && angleDeg >= step) {
//            WristServo.setPosition(1);
//            telemetry.addData("Going to POS", "LEFT DIA");
//        } else if (angleDeg <= 22.5 && angleDeg > - 22.5){
//            WristServo.setPosition(0.15);
//            telemetry.addData("Going to POS", "HORIZONTAL");
//        }else {
//            WristServo.setPosition(0);
//            telemetry.addData("Going to POS", "RIGHT DIA");
//        }
        double wristAngle = 60 + angleDeg;
        WristServo.setPosition(wristAngle /300);
        telemetry.addData("CLAW POSITION", wristAngle/300);
    }
    private void Closeclaw(){
            telemetry.addData("MOVING CLAW: ", "CLOSE");
            ClawServo.setPosition(0.8); //close position
    }
    private void OpenClaw(){
        telemetry.addData("MOVING CLAW: ", "OPEN");
        ClawServo.setPosition(1); //open position
    }
    // Robot hardware
    private RobotHardware robot;
    
    // Color blob detector
    private ColorBlobDetector colorDetector;
    public double turretMovementAngle = 0;
    public boolean moveShoulderUp = false;
    public double verticalDistance = -1;
    public double horizontalDistance = -1;
    public double orientation = -100;
    
    // Camera position and calibration parameters
    // These values should be measured for your specific robot setup
    private static final double CAMERA_HEIGHT_INCHES = 10.75;  // Height of camera from ground
    public static double CAMERA_FORWARD_OFFSET = 1; // Distance forward from robot center //8
    private static final double CAMERA_HORIZONTAL_OFFSET = -11; // Centered horizontally //-5.25
    private static final double CAMERA_HORIZONTAL_FOV_DEGREES = 48.8; // Horizontal field of view
    private static final double CAMERA_VERTICAL_FOV_DEGREES = 45.0;   // Vertical field of view
    public static double ARMLENGTH = 7.25;
    public final double TICKSPERINCHES = 538/(1.575*Math.PI); //Total ticks to extend slide divided by slide length
    public DcMotorEx SlideMotor;
    public static double velocity = 100;
    @Override
    public void runOpMode() {
        pickUpPivot = hardwareMap.get(Servo.class, "PickupPivot");
        ShoulderServo = hardwareMap.get(Servo.class, "ShoulderServo");
        ClawServo = hardwareMap.get(Servo.class, "ClawServo");
        WristServo = hardwareMap.get(Servo.class, "WristServo");
        turret = hardwareMap.get(Servo.class, "TurretServo");
        SlideMotor = hardwareMap.get(DcMotorEx.class, "SlideMotor");
        SlideMotor.setTargetPosition(0);
        SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Initialize telemetry for user feedback
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        
        // Initialize robot hardware
//        robot = new RobotHardware(hardwareMap);

        // Initialize color blob detector with camera position parameters
        // This is critical for accurate real-world measurements from the robot's center
        colorDetector = new ColorBlobDetector(
            hardwareMap,
            telemetry,
            ColorBlobDetector.getRedTarget(), // Start with RED detection
            CAMERA_HEIGHT_INCHES,
            CAMERA_FORWARD_OFFSET,
            CAMERA_HORIZONTAL_OFFSET,
            CAMERA_HORIZONTAL_FOV_DEGREES,
            CAMERA_VERTICAL_FOV_DEGREES
        );
        
        telemetry.addData("Status", "Initialized. Press play to start.");
        telemetry.update();
        OpenClaw();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("HORIZONTZAL DISTANCE: ", horizontalDistance);
            telemetry.addData("VERTICAL DISTANCE: ", verticalDistance);
            telemetry.addData("SAMPLE OREINTATION: ", orientation);
            telemetry.addData("TURRET ANGLE: ", turretMovementAngle);
            pickUpPivot.setPosition(0.55);
            // Process controller inputs for changing target color
            handleControls();
            
            // Get the number of detected objects
            int blobCount = colorDetector.getBlobCount();
            
            // Display basic information
            telemetry.addData("Status", "Running");
            telemetry.addData("Objects Detected", blobCount);
            
            // If objects are detected, show detailed information
            if (blobCount > 0) {
                // Get the largest detected object
                ColorBlobLocatorProcessor.Blob largestBlob = colorDetector.getLargestBlob();
                
                if (largestBlob != null) {
                    // Calculate real-world measurements
                    double[] measurements = colorDetector.calculateDistanceAndAngle(largestBlob);
                    double angle = measurements[1];    // Angle from robot's forward direction
                    double height = measurements[2];   // Height from ground
                    double diagnalDistance = measurements[5];
                    if(verticalDistance == -1 && horizontalDistance == -1 && orientation == -100){
                        //Only calculate the distances once
                        orientation = measurements[3]; // Orientation angle of the rectangle
                        verticalDistance = changeVerticalDepth(measurements[0]); // Distance from robot center
                        horizontalDistance = changeHorizontalDepth(measurements[4]); // the horizontal distance
                    }
                    telemetry.addLine("\n--- Largest Object Details ---");
                    telemetry.addData("CURRENT SERVO POSITION: ", turret.getPosition() * 300);
                    telemetry.addData("DIAGNAL DISTANCE: ", diagnalDistance);
                    telemetry.addData("Size", String.format("%.2f sq in", largestBlob.getContourArea() / 100.0));
                    telemetry.addData("Angle from Robot", String.format("%.2f degrees", angle));
                    telemetry.addData("Height", String.format("%.2f inches", height));
                    MoveSlideandTurret(verticalDistance, horizontalDistance); //Move the slide and turret to the desired position
                    if(!SlideMotor.isBusy()) {
                        MoveShoulder(); //Move the shoulder down
                        try{
                            Thread.sleep(wait);
                        }
                        catch (InterruptedException e){
                            telemetry.addData("GOT ERROR: ", e);
                        }
                        Closeclaw(); // Close the claw to pickup sample
                    }
                    rotateWristServo(orientation); // Move the claw Servo to sample Orientation
                    //moveShoulderUp = true;
                    //MoveSlide(testDepth, angle);
                    // Display detailed information about the object

                    // Example of how to use this data for robot control
                    telemetry.addLine("\n--- Navigation Example ---");
                    
                    // Determine if the robot is aligned with the object
                    boolean isAligned = Math.abs(angle) < 5.0; // Within 5 degrees
                    telemetry.addData("Robot Aligned", isAligned ? "Yes" : "No");
                    
                    // Determine if the robot is at the desired distance
                    double desiredDistance = 12.0; // 12 inches
                    double distanceError = verticalDistance - desiredDistance;
                    boolean isAtDistance = Math.abs(distanceError) < 2.0; // Within 2 inches
                    telemetry.addData("At Desired Distance", isAtDistance ? "Yes" : "No");
                    
                    // Determine if the rectangle is oriented horizontally
                    // This is useful for tasks where the orientation of the object matters
                    boolean isHorizontal = Math.abs(orientation) < 10.0; // Within 10 degrees of horizontal
                    telemetry.addData("Rectangle Orientation", isHorizontal ? "Horizontal" : "Rotated");
                    
                    // Example navigation instructions
                    if (!isAligned) {
                        // Need to rotate to align with the object
                        if (angle > 0) {
                            telemetry.addData("Navigation", "Turn RIGHT to align");
                        } else {
                            telemetry.addData("Navigation", "Turn LEFT to align");
                        }
                    } else if (!isAtDistance) {
                        // Need to move forward/backward to reach desired distance
                        if (distanceError > 0) {
                            telemetry.addData("Navigation", "Move FORWARD to reach target");
                        } else {
                            telemetry.addData("Navigation", "Move BACKWARD to reach target");
                        }
                    } else if (!isHorizontal) {
                        // Need to adjust to match the object's orientation
                        telemetry.addData("Navigation", String.format("Object rotated %.1f degrees from horizontal", orientation));
                    } else {
                        // Robot is aligned, at the correct distance, and object is horizontal
                        telemetry.addData("Navigation", "ALIGNED, at CORRECT DISTANCE, and HORIZONTAL");
                    }
                    
                    // Example of how to use this data for autonomous control
                    // (commented out since this is just a demo)
                    /*
                    if (!isAligned) {
                        // Turn to align with the object
                        double turnPower = 0.3 * Math.signum(angle);
                        robot.setDrivePowers(turnPower, -turnPower, turnPower, -turnPower);
                    } else if (!isAtDistance) {
                        // Move forward/backward to reach desired distance
                        double drivePower = 0.3 * Math.signum(distanceError);
                        robot.setDrivePowers(drivePower, drivePower, drivePower, drivePower);
                    } else {
                        // Stop when aligned and at correct distance
                        robot.setDrivePowers(0, 0, 0, 0);
                    }
                    */
                }
            } else {
                telemetry.addLine("No objects detected. Adjust camera or change target color.");
            }
            
            // Display controls
            telemetry.addLine("\n--- Controls ---");
            telemetry.addData("X / B / Y / A", "Switch to RED / BLUE / GREEN / YELLOW detection");
            telemetry.addData("DPAD Up/Down", "Resume/Stop camera streaming");
            
            // Update telemetry
            telemetry.update();
            
            // Sleep to prevent CPU overuse
            sleep(50);
        }
        
        // Clean up resources
        if (colorDetector != null) {
            colorDetector.close();
        }
    }
    
    /**
     * Handle controller inputs for changing vision settings
     */
    private void handleControls() {
        // Toggle between different color targets
        if (gamepad1.x) {
            // X button switches to RED detection
            colorDetector.setTargetColor(ColorBlobDetector.getRedTarget());
            telemetry.addData("Color Target", "RED");
        } else if (gamepad1.b) {
            // B button switches to BLUE detection
            colorDetector.setTargetColor(ColorBlobDetector.getBlueTarget());
            telemetry.addData("Color Target", "BLUE");
        } else if (gamepad1.y) {
            // Y button switches to GREEN detection
            colorDetector.setTargetColor(ColorBlobDetector.getGreenTarget());
            telemetry.addData("Color Target", "GREEN");
        } else if (gamepad1.a) {
            // A button switches to YELLOW detection
            colorDetector.setTargetColor(ColorBlobDetector.getYellowTarget());
            telemetry.addData("Color Target", "YELLOW");
        }
        
        // Toggle camera streaming
        if (gamepad1.dpad_down) {
            colorDetector.pauseStreaming();
            telemetry.addData("Camera", "Stopped");
        } else if (gamepad1.dpad_up) {
            colorDetector.resumeStreaming();
            telemetry.addData("Camera", "Streaming");
        }
    }
}
