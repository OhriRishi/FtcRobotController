package org.firstinspires.ftc.teamcode.drive.opmode;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.io.File;
import java.util.Scanner;

public class UndistortPreProcessor extends OpenCvPipeline {
    private final Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
    private final Mat distCoeffs = new Mat(5, 1, CvType.CV_64F);
    private boolean calibrationLoaded = false;

    public void UndistortPreProcesso() {
        try {
            File file = new File("/sdcard/FIRST/calibration/camera_calibration.txt");
            Scanner scanner = new Scanner(file);
            scanner.nextLine(); // camera_matrix:
            for (int i = 0; i < 3; i++) {
                String[] row = scanner.nextLine().trim().split("\\s+");
                for (int j = 0; j < 3; j++) {
                    cameraMatrix.put(i, j, Double.parseDouble(row[j]));
                }
            }
            scanner.nextLine(); // distortion_coeffs:
            String[] coeffs = scanner.nextLine().trim().split("\\s+");
            for (int i = 0; i < coeffs.length; i++) {
                distCoeffs.put(i, 0, Double.parseDouble(coeffs[i]));
            }
            calibrationLoaded = true;
        } catch (Exception e) {
            calibrationLoaded = false;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        if (!calibrationLoaded) return input;
        Mat undistorted = new Mat();
        Imgproc.undistort(input, undistorted, cameraMatrix, distCoeffs);
        return undistorted;
    }
}
