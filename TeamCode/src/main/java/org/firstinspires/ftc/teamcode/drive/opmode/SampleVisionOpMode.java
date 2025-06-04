
package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.RotatedRect;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
@Config
@TeleOp(name="Sample Detection Vision", group="Vision")
public class SampleVisionOpMode extends LinearOpMode {
    public static int IMAGE_HIEGHT = 640;
    public static int IMAGE_WIDTH = 480;

    OpenCvWebcam webcam;
    SampleDetectionPipeline pipeline;

    @Override
    public void runOpMode() {
        int camMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), camMonitorViewId);

        pipeline = new SampleDetectionPipeline();
        pipeline.setColorMode(SampleDetectionPipeline.ColorMode.RED);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            //override onOpened to start streaming
            @Override
            public void onOpened() {
                webcam.startStreaming(IMAGE_HIEGHT, IMAGE_WIDTH, OpenCvCameraRotation.UPRIGHT);
            }
            //Handle error handling
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Got error number: ", errorCode);
            }
        });

        telemetry.addLine("Use D-pad up/down/left to change color mode:");
        telemetry.addLine("UP = RED, LEFT = BLUE, DOWN = YELLOW");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                pipeline.setColorMode(SampleDetectionPipeline.ColorMode.RED);
            } else if (gamepad1.dpad_left) {
                pipeline.setColorMode(SampleDetectionPipeline.ColorMode.BLUE);
            } else if (gamepad1.dpad_down) {
                pipeline.setColorMode(SampleDetectionPipeline.ColorMode.YELLOW);
            }

            telemetry.addData("Detected", pipeline.latestRects != null ? pipeline.latestRects.length : 0);
            for (int i = 0; i < pipeline.latestDistances.length; i++) {
                double[] dist = pipeline.latestDistances[i];
                if (dist == null || dist.length < 3) continue;

                telemetry.addData("ANGLE: ", dist[1]);
                telemetry.addData("X: ", dist[0]);
                telemetry.addData("Y: ", dist[2]);
            }
            telemetry.update();
            sleep(100);
        }
    }
}
