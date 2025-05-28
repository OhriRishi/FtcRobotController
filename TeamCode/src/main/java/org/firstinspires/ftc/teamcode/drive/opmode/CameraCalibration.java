package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

@Config
@TeleOp
public class CameraCalibration extends LinearOpMode {
    public double verticalDepth;
    public double orientation;
    public double horizontalDepth;
    private ColorBlobDetector Detector;
    public double verticalDepthSlope = 1;
    public double verticalDepthIntercept = 0;
    public double horizontalDepthSlope = 1;
    public double horizontalDepthIntercept = 0;
    private static final double CAMERA_HEIGHT_INCHES = 10.75;  // Height of camera from ground
    private static final double CAMERA_FORWARD_OFFSET = 0.0; // Distance forward from robot center //8
    private static final double CAMERA_HORIZONTAL_OFFSET = -11; // Centered horizontally //-5.25
    private static final double CAMERA_HORIZONTAL_FOV_DEGREES = 48.8; // Horizontal field of view
    private static final double CAMERA_VERTICAL_FOV_DEGREES = 45.0;   // Vertical field of view
    public double changeVerticalDepth(double depth){
        return (verticalDepthSlope * depth) + verticalDepthIntercept;
    }
    public double changeHorizontalDepth(double depth){
        return (horizontalDepthSlope * depth) + horizontalDepthIntercept;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Detector = new ColorBlobDetector(
                hardwareMap,
                telemetry,
                ColorBlobDetector.getRedTarget(), // Start with RED detection
                CAMERA_HEIGHT_INCHES,
                CAMERA_FORWARD_OFFSET,
                CAMERA_HORIZONTAL_OFFSET,
                CAMERA_HORIZONTAL_FOV_DEGREES,
                CAMERA_VERTICAL_FOV_DEGREES
        );
        waitForStart();
        while(opModeIsActive()){
            int blobCount = Detector.getBlobCount();
            if (blobCount > 0) {
                // Get the largest detected object
                ColorBlobLocatorProcessor.Blob largestBlob = Detector.getLargestBlob();

                if (largestBlob != null) {
                    double[] returnedValues = Detector.calculateDistanceAndAngle(largestBlob);
                    verticalDepth = changeVerticalDepth(returnedValues[0]);
                    horizontalDepth = changeHorizontalDepth(returnedValues[4]);
                    orientation = returnedValues[3];
                }
            }

            telemetry.addData("Vertical Depth", verticalDepthSlope * verticalDepth + verticalDepthIntercept);
            telemetry.addData("Horizontal Depth", -horizontalDepthSlope*horizontalDepth + horizontalDepthIntercept);
            telemetry.addData("Orientation", orientation);

            telemetry.update();
        }
    }
}
