package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Comparator;
@TeleOp
public class visionOpmode extends LinearOpMode {
    public SampleDetectionPipelineV2 sampleDetectionPipeline;
    //    public TensorFlow VisionCalibration;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        int camMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), camMonitorViewId);
        sampleDetectionPipeline = new SampleDetectionPipelineV2();
        webcam.setPipeline(sampleDetectionPipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            //override onOpened to start streaming
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            //Handle error handling
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Got error number: ", errorCode);
            }
        });
//        VisionCalibration = new TensorFlow();
        waitForStart();
        while(opModeIsActive()){
            if (sampleDetectionPipeline.latestRects != null && sampleDetectionPipeline.latestDistances != null) {

                for (int loop = 0; loop < sampleDetectionPipeline.latestRects.length; loop++) {
                    double realX = sampleDetectionPipeline.GetRealXinches(loop);
                    double realY = sampleDetectionPipeline.GetRealYinches(loop);
                    double realOrientation = sampleDetectionPipeline.GetRealSampleOrientation(loop);
                    telemetry.addData("GOT REAL X: ", realX);
                    telemetry.addData("GOT DEPTH: ", realY);
                    telemetry.addData("GOT ORIENTATION: ", realOrientation);
                    telemetry.update();
//                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Real Vertical: " + realY);
//                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Real Horizontal: " + realX);
//                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Real Orientation: " + realOrientation);

//                    TensorFlow.CalibrationResult result = VisionCalibration.calibrate((float) (realX - RobotConstants.TURRET_OFFSET_FROM_CAMERA), (float) realY, (float) realOrientation);

//                    double calibratedYOffset = result.calibratedY;
//                    double calibratedXOffset = result.calibratedX;
//                    double calibratedSampleOrientation = result.calibratedAngle;
//
//                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Calibrated Vertical: " + calibratedYOffset);
//                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Calibrated Horizontal: " + calibratedXOffset);
//                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Calibrated Orientation: " + calibratedSampleOrientation);
                }
            }
        }
    }
}
