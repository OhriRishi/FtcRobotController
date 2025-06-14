package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
@Disabled
public class LimeLightCode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //Set the limelight pipeline to 0
        limelight.pipelineSwitch(0);
        limelight.start();
        waitForStart();
        while(opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double[] CustomResult = result.getPythonOutput();
                double tx = result.getTx(); // horizontal offset
                double ty = result.getTy(); // vertical offset
                double distance = CustomResult[5]; // sample rotation angle
                double angle = CustomResult[3]; // bounding box width, etc.
                double countorCount = CustomResult[4];
                double centerX = CustomResult[1];
                double centerY = CustomResult[2];
                telemetry.addData("Horizontal Angle: ", tx);
                telemetry.addData("Vertical Angle: ", ty);
                telemetry.addData("Vertical Distance: ", distance);
                telemetry.addData("Sample orientation: ", angle);
                telemetry.addData("Number of samples found: ", countorCount);
                telemetry.addData("Pixel center X: ", centerX);
                telemetry.addData("Pixel center Y: ", centerY);
            }
        }
        telemetry.update();
    }
}
