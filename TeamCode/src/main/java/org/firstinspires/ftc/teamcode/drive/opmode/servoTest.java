package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp
@Config
public class servoTest extends LinearOpMode {
    //claw close is 0.95 and open is 0.8\
    //5 turn servo 0.35 is to pass to transfer and 0.5 is pickup
    //public CRServo axon;

    //public static double power = 1;
    public Servo servo;
    public static double position = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "axon");
        waitForStart();
        while(opModeIsActive()){
            servo.setPosition(position);
        }
    }
}
