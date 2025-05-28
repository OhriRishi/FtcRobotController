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
@TeleOp(name = "Turret Test", group = "Demo")
public class TurretTest extends LinearOpMode {
    private Servo turret;

    public static double YOffset;
    public static double XOffset;
    double NewY = Math.sqrt(ARMLENGTH*ARMLENGTH - XOffset * XOffset);



    private double changeVerticalDepth(double depth){
        return 0.129 * depth + 6.07;
    }
    private double changeHorizontalDepth(double depth){
        return -0.316*depth + 1.52;
    }
    public void MoveTurret(double YOffset, double XOffset) {
//        turret.setPosition(0.5 + (turretMovement/300));
//        try {
//            Thread.sleep(100);
//        }
//        catch (InterruptedException e){
//            telemetry.addData("Error: ", e);
//        }

            double NewY = Math.sqrt(ARMLENGTH*ARMLENGTH - XOffset * XOffset);
            double turretMovement = Math.toDegrees(Math.atan(XOffset/NewY));
            turret.setPosition(0.5 + (turretMovement/300));
            moveShoulderUp = false; // Give the command to move the shoulder down
            telemetry.addData("Turret Angle: ", turretMovement);
            telemetry.addData("Turret Position: ", 0.5 + ((turretMovement)/300));
            telemetry.addData("Current Servo Pos: ", turret.getPosition());
        telemetry.update();
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
    private static final double CAMERA_FORWARD_OFFSET = 0.0; // Distance forward from robot center //8
    private static final double CAMERA_HORIZONTAL_OFFSET = -11; // Centered horizontally //-5.25
    private static final double CAMERA_HORIZONTAL_FOV_DEGREES = 48.8; // Horizontal field of view
    private static final double CAMERA_VERTICAL_FOV_DEGREES = 45.0;   // Vertical field of view
    public static double ARMLENGTH = 6.5;
    public final double TICKSPERINCHES = 538/(1.575*Math.PI); //Total ticks to extend slide divided by slide length
    public DcMotorEx SlideMotor;
    public static double velocity = 100;
    @Override
    public void runOpMode() {

        turret = hardwareMap.get(Servo.class, "TurretServo");

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
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("HORIZONTAL DISTANCE: ", horizontalDistance);
            telemetry.addData("VERTICAL DISTANCE: ", verticalDistance);
            telemetry.addData("SAMPLE OREINTATION: ", orientation);
            telemetry.addData("TURRET ANGLE: ", turretMovementAngle);
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
                    MoveTurret(verticalDistance, horizontalDistance);
                    telemetry.addLine("\n--- Largest Object Details ---");
                    telemetry.addData("CURRENT SERVO POSITION: ", turret.getPosition() * 300);
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
