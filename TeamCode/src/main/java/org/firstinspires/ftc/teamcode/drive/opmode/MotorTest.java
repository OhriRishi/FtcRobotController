package org.firstinspires.ftc.teamcode.drive.opmode;

import android.transition.Slide;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Config
@TeleOp(name="MotorTest", group="Linear OpMode")
public class MotorTest extends LinearOpMode {
    public DcMotorEx SlideMotor;
    public final double TICKSPERINCHES = 538/(1.575*Math.PI);
    public static int Inches = 10;
    public static double velocity = 100;
    public Servo turret;
    public static double YOffset = 10;
    public static double XOffset = -5;
    public static double ANGLETOMOVE = 40;
    public static double ARMLENGTH = 7;
    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotorEx Vbar;
//
//    public static int pos = 0;
//
//    public static double velocity = 100;

    @Override
    public void runOpMode() {
        turret = hardwareMap.get(Servo.class, "TurretServo");
        SlideMotor = hardwareMap.get(DcMotorEx.class, "SlideMotor");
        SlideMotor.setTargetPosition(0);
        SlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
//        Vbar = hardwareMap.get(DcMotorEx.class,"Vbar");
//        Vbar.setDirection(DcMotorSimple.Direction.REVERSE);
//        Vbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Vbar.setTargetPosition(0);
//        Vbar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Vbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            MoveSlide(YOffset, XOffset);
//                MoveSlide(Inches);
        }
    }

    public void MoveSlide(double YOffset, double XOffset) {
        double NewY = Math.sqrt(ARMLENGTH*ARMLENGTH - XOffset * XOffset);
        double SlideMove = YOffset - NewY;
        double turretMovement = Math.toDegrees(Math.atan(XOffset/NewY));
        SlideMotor.setTargetPosition((int) (SlideMove * TICKSPERINCHES));
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideMotor.setVelocity(velocity);
        telemetry.addData("Moving slide Inches: ", (int) (SlideMove * TICKSPERINCHES));
        if (!SlideMotor.isBusy()) {
                turret.setPosition(0.5 - (turretMovement/300));
                telemetry.addData("Turret Angle: ", turretMovement);
                telemetry.addData("Turret Position: ", 0.5 - ((turretMovement)/300));
            telemetry.addData("Current Servo Pos: ", turret.getPosition());
        }
        telemetry.update();
    }
}
