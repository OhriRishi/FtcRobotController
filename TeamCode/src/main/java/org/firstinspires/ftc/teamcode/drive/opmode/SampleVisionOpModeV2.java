//package org.firstinspires.ftc.teamcode.drive.opmode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//import org.opencv.core.RotatedRect;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//
//
//@Config
//@TeleOp(name="Sample Detection Vision V2", group="Vision")
//public class SampleVisionOpModeV2 extends LinearOpMode {
//    public static int IMAGE_WIDTH = 640;
//    public static int IMAGE_HIEGHT = 480;
//
//    public TensorFlow VisionCalibration;
//
//    OpenCvWebcam webcam;
//    SampleDetectionPipelineV2 pipeline;
//
//    @Override
//    public void runOpMode() {
//        VisionCalibration = new TensorFlow();
//        int camMonitorViewId = hardwareMap.appContext.getResources()
//                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), camMonitorViewId);
//
//        pipeline = new SampleDetectionPipelineV2();
//        pipeline.setColorMode(SampleDetectionPipelineV2.ColorMode.RED);
//        webcam.setPipeline(pipeline);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            //override onOpened to start streaming
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(IMAGE_WIDTH,IMAGE_HIEGHT, OpenCvCameraRotation.UPRIGHT);
//            }
//            //Handle error handling
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Got error number: ", errorCode);
//            }
//        });
//
//        telemetry.addLine("Use D-pad up/down/left to change color mode:");
//        telemetry.addLine("UP = RED, LEFT = BLUE, DOWN = YELLOW");
//        telemetry.update();
//        waitForStart();
//
//        while (opModeIsActive()) {
//            if (gamepad1.dpad_up) {
//                pipeline.setColorMode(SampleDetectionPipelineV2.ColorMode.RED);
//            } else if (gamepad1.dpad_left) {
//                pipeline.setColorMode(SampleDetectionPipelineV2.ColorMode.BLUE);
//            } else if (gamepad1.dpad_down) {
//                pipeline.setColorMode(SampleDetectionPipelineV2.ColorMode.YELLOW);
//            }
//
//            telemetry.addData("Detected", pipeline.latestRects != null ? pipeline.latestRects.length : 0);
//            if (pipeline.latestRects != null && pipeline.latestDistances != null) {
//                for (int i = 0; i < pipeline.latestRects.length; i++) {
//                    TensorFlow.CalibrationResult result = VisionCalibration.calibrate((float) pipeline.GetRealXinches(i), (float) pipeline.GetRealYinches(i), (float) pipeline.GetSampleOrientation(i));
//                    RotatedRect rect = pipeline.latestRects[i];
//                    telemetry.addData("GOT CENTER Y: ", rect.center.y);
//                    telemetry.addData("GOT CENTER X:", rect.center.x);
//                    telemetry.addData("GOT VERTICAL DISTANCE FROM ROBOT: ", result.realY);
//                    telemetry.addData("GOT HORIONTAL DISTANCE FROM ROBOT: ", result.realX);
//                    telemetry.addData("GOT RECTANGLE OREINTATION: ", result.realAngle);
//                    telemetry.addData("GOT ANGLE FROM ROBOT: ", pipeline.GetSampleAngleFromRobot(i));
//                }
//            }
//            telemetry.update();
//            sleep(100);
//        }
//    }
//}