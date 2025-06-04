package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "moveWithoutImage", group = "Linear Opmode")
public class moveWithXAndY extends LinearOpMode {
    public void MoveSlideandTurret(double YOffset, double XOffset) {
        double NewY = Math.sqrt(ARMLENGTH * ARMLENGTH - XOffset * XOffset);
        double SlideMove = YOffset - NewY;
        turretMovementAngle = Math.toDegrees(Math.atan(XOffset / NewY));
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
            turret.setPosition(0.5 + (turretMovementAngle / 300));
            moveShoulderUp = false; // Give the command to move the shoulder down
            telemetry.addData("Turret Angle: ", turretMovementAngle);
            telemetry.addData("Turret Position: ", 0.5 - ((turretMovementAngle) / 300));
            telemetry.addData("Current Servo Pos: ", turret.getPosition());
        }
        telemetry.update();
    }

    public void MoveShoulder() {
        telemetry.addData("MOVING SHOULDER:", "DOWN");
        ShoulderServo.setPosition(0.47); //pickup position
    }

    public void MoveElbow() {
        telemetry.addData("MOVING ELBOW: ", "DOWN");
        ElbowServo.setPosition(0.1);
    }

    private void rotateWristServo(double angleDeg) {
        telemetry.addData("MOVING WRIST SERVO: ", "MOVING");
        angleDeg -=90; //wrist has to be ppp to the sample
        angleDeg -= turretMovementAngle;
        telemetry.addData("ANGLE DEGREES: ", angleDeg);
        telemetry.addData("TURRET MOVEMENT ANGLE: ", turretMovementAngle);
        double wristAngle = angleDeg + (WristCenter * 300);
        WristServo.setPosition(wristAngle / 300);
        telemetry.addData("CLAW POSITION", wristAngle / 300);
    }

    private void Closeclaw() {
        telemetry.addData("MOVING CLAW: ", "CLOSE");
        ClawServo.setPosition(0.8); //close position
    }

    private void OpenClaw() {
        telemetry.addData("MOVING CLAW: ", "OPEN");
        ClawServo.setPosition(1); //open position
    }

    public static long wait = 2000;
    public static double WristCenter = 0.52;
    public double turretMovementAngle = 0;
    public boolean moveShoulderUp = false;
    public static double orientation = -100;
    public static double ARMLENGTH = 7;
    public Servo pickUpPivot;
    public static double TurretCenter = 0.475;
    public Servo ShoulderServo;
    public Servo ClawServo;
    public Servo WristServo;
    public Servo ElbowServo;
    private Servo turret;
    public final double TICKSPERINCHES = 538 / (1.575 * Math.PI); //Total ticks to extend slide divided by slide length
    public DcMotorEx SlideMotor;
    public static double velocity = 100;
    public static double verticalDistance = 10;
    public static double horizontalDistance = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ShoulderServo = hardwareMap.get(Servo.class, "ShoulderServo");
        ClawServo = hardwareMap.get(Servo.class, "ClawServo");
        WristServo = hardwareMap.get(Servo.class, "WristServo");
        turret = hardwareMap.get(Servo.class, "TurretServo");
        ElbowServo = hardwareMap.get(Servo.class, "ElbowServo");
        SlideMotor = hardwareMap.get(DcMotorEx.class, "SlideMotor");
        SlideMotor.setTargetPosition(0);
        SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Initialize telemetry for user feedback
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        OpenClaw();
        waitForStart();

        int step = 0;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive()) {

            switch (step) {
                case 0:
                    // Move Slides, Elbow, and Wrist together
                    MoveSlideandTurret(verticalDistance, horizontalDistance); // Starts slide motion
                    MoveElbow(); // Set elbow down immediately
                    rotateWristServo(orientation); // Adjust wrist orientation
                    step++;
                    break;

                case 1:
                    // Wait for slide to finish
                    if (!SlideMotor.isBusy()) {
                        step++;
                    }
                    break;

                case 2:
                    // Move Turret once slide is done
                    double NewY = Math.sqrt(ARMLENGTH * ARMLENGTH - horizontalDistance * horizontalDistance);
                    turretMovementAngle = Math.toDegrees(Math.atan(horizontalDistance / NewY));
                    turret.setPosition(TurretCenter + (turretMovementAngle / 300.0));
                    telemetry.addData("Turret Angle", turretMovementAngle);
                    telemetry.addData("Turret Pos", turret.getPosition());
                    timer.reset();
                    step++;
                    break;

                case 3:
                    // Delay before moving shoulder (if needed)
                    if (timer.milliseconds() > 300) {
                        MoveShoulder(); // Move shoulder down
                        timer.reset();
                        step++;
                    }
                    break;

                case 4:
                    // Wait a bit then close the claw
                    if (timer.milliseconds() > wait) {
                        Closeclaw(); // Grab object
                        step++;
                    }
                    break;

                default:
                    telemetry.addData("Sequence", "Complete");
                    break;
//            MoveSlideandTurret(verticalDistance, horizontalDistance); //Move the slide and turret to the desired position
//            if(!SlideMotor.isBusy()) {
//                MoveShoulder(); //Move the shoulder down
//                try{
//                    Thread.sleep(wait);
//                }
//                catch (InterruptedException e){
//                    telemetry.addData("GOT ERROR: ", e);
//                }
//                Closeclaw(); // Close the claw to pickup sample
//            }
//            rotateWristServo(orientation); // Move the claw Servo to sample Orientation
//            MoveElbow();
            }
            telemetry.update();
        }
    }
}
