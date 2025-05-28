package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.*;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

@TeleOp(name = "Camera Calibration OpMode")
public class CameraCalibrationPics extends LinearOpMode {
    OpenCvCamera camera;
    final List<Mat> savedFrames = new ArrayList<>();
    final int requiredFrames = 10;
    int frameCount = 0;

    final Size patternSize = new Size(9, 6); // inner corners of chessboard
    final double squareSize = 25.0; // in mm or chosen units

    boolean captureRequested = false;

    @Override
    public void runOpMode() {
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1")
        );
        camera.setPipeline(new CalibrationPipeline());

        telemetry.addLine("Opening camera...");
        telemetry.update();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera failed to open. Error code:", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("Press [A] to capture chessboard frames.");
        telemetry.addLine("Move board between each capture.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && frameCount < requiredFrames) {
            if (gamepad1.a) {
                captureRequested = true;
            }

            telemetry.addData("Captured Frames", frameCount + "/" + requiredFrames);
            telemetry.addLine("Press [A] to capture a frame.");
            telemetry.update();
            sleep(100); // debounce
        }

        if (savedFrames.size() >= requiredFrames) {
            calibrateCamera(savedFrames, patternSize, squareSize);
        } else {
            telemetry.addLine("Calibration failed: Not enough good frames.");
            telemetry.update();
        }

        camera.stopStreaming();
    }

    class CalibrationPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Mat gray = new Mat();
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
            MatOfPoint2f corners = new MatOfPoint2f();

            boolean found = Calib3d.findChessboardCorners(gray, patternSize, corners);

            if (found) {
                Imgproc.cornerSubPix(
                        gray,
                        corners,
                        new Size(11, 11),
                        new Size(-1, -1),
                        new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.1)
                );

                Calib3d.drawChessboardCorners(input, patternSize, corners, found);

                if (captureRequested && frameCount < requiredFrames) {
                    savedFrames.add(input.clone());
                    frameCount++;
                    captureRequested = false;
                }
            } else {
                captureRequested = false;
            }

            return input;
        }
    }

    void calibrateCamera(List<Mat> frames, Size boardSize, double squareSize) {
        List<MatOfPoint2f> imagePoints = new ArrayList<>();
        List<MatOfPoint3f> objectPoints = new ArrayList<>();

        // Generate 3D object points (same for all frames)
        MatOfPoint3f obj = new MatOfPoint3f();
        List<Point3> objPts = new ArrayList<>();
        for (int i = 0; i < boardSize.height; i++) {
            for (int j = 0; j < boardSize.width; j++) {
                objPts.add(new Point3(j * squareSize, i * squareSize, 0));
            }
        }
        obj.fromList(objPts);

        // Detect corners in each frame
        for (Mat frame : frames) {
            Mat gray = new Mat();
            Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGB2GRAY);
            MatOfPoint2f corners = new MatOfPoint2f();
            boolean found = Calib3d.findChessboardCorners(gray, boardSize, corners);

            if (found) {
                Imgproc.cornerSubPix(
                        gray,
                        corners,
                        new Size(11, 11),
                        new Size(-1, -1),
                        new TermCriteria(TermCriteria.EPS + TermCriteria.MAX_ITER, 30, 0.1)
                );
                imagePoints.add(corners);
                objectPoints.add(obj);
            }
        }

        if (imagePoints.size() < 3) {
            telemetry.addLine("Calibration aborted: not enough valid images.");
            telemetry.update();
            return;
        }

        Mat cameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
        Mat distCoeffs = Mat.zeros(8, 1, CvType.CV_64F);
        Size imageSize = frames.get(0).size();

        // Convert List<MatOfPointXf> to List<Mat> as required by calibrateCamera
        List<Mat> objectPointsMat = new ArrayList<>();
        for (MatOfPoint3f m : objectPoints) objectPointsMat.add(m);

        List<Mat> imagePointsMat = new ArrayList<>();
        for (MatOfPoint2f m : imagePoints) imagePointsMat.add(m);

        Calib3d.calibrateCamera(objectPointsMat, imagePointsMat, imageSize, cameraMatrix, distCoeffs, new ArrayList<>(), new ArrayList<>());

        telemetry.addLine("Calibration complete!");
        telemetry.addData("Camera Matrix", cameraMatrix.dump());
        telemetry.addData("Distortion Coeffs", distCoeffs.dump());
        telemetry.update();

        saveCalibrationData(cameraMatrix, distCoeffs);
    }

    private void saveCalibrationData(Mat cameraMatrix, Mat distCoeffs) {
        try {
            File dir = new File("/sdcard/FIRST/calibration");
            if (!dir.exists()) dir.mkdirs();

            File file = new File(dir, "camera_calibration.txt");
            FileWriter writer = new FileWriter(file);

            writer.write("camera_matrix:\n");
            for (int i = 0; i < cameraMatrix.rows(); i++) {
                for (int j = 0; j < cameraMatrix.cols(); j++) {
                    writer.write(cameraMatrix.get(i, j)[0] + " ");
                }
                writer.write("\n");
            }

            writer.write("\ndistortion_coeffs:\n");
            for (int i = 0; i < distCoeffs.rows(); i++) {
                writer.write(distCoeffs.get(i, 0)[0] + " ");
            }

            writer.close();
            telemetry.addLine("Calibration data saved.");
            telemetry.addData("Path", file.getAbsolutePath());
            telemetry.update();
        } catch (IOException e) {
            telemetry.addLine("Failed to save calibration data.");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        }
    }
}

