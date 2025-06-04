package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.opmode.DetectionParameters;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Core sample color detection utility class
 * Provides static methods for color detection and pose estimation
 * This class contains the main computer vision algorithms
 */
public class SampleColorDetector {

    /**
     * Detect samples in an image using color-based detection
     *
     * @param inputFrame The input image frame
     * @param params Detection parameters
     * @return List of detected sample poses
     */
    public static List<SamplePose> detectSamples(Mat inputFrame, DetectionParameters params) {
        List<SamplePose> detectedSamples = new ArrayList<>();

        // Working matrices
        Mat hsvMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphMat = new Mat();
        Mat hierarchy = new Mat();

        try {
            // Step 1: Convert to HSV color space
            Imgproc.cvtColor(inputFrame, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Step 2: Apply color thresholding
            applyColorThresholding(hsvMat, thresholdMat, params);

            // Step 3: Apply morphological operations
            applyMorphologicalOperations(thresholdMat, morphMat, params);

            // Step 4: Find and process contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(morphMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Step 5: Process each contour
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);

                // Filter by area
                if (area >= params.minContourArea && area <= params.maxContourArea) {
                    SamplePose pose = processContour(contour, inputFrame.size(), params);
                    if (pose != null && pose.confidence >= params.minConfidence) {
                        detectedSamples.add(pose);
                    }
                }
            }

            // Release contour memory
            for (MatOfPoint contour : contours) {
                contour.release();
            }

        } finally {
            // Release matrices
            hsvMat.release();
            thresholdMat.release();
            morphMat.release();
            hierarchy.release();
        }

        return detectedSamples;
    }

    /**
     * Apply color thresholding based on current color mode
     */
    private static void applyColorThresholding(Mat hsvMat, Mat thresholdMat, DetectionParameters params) {
        Scalar[] ranges = params.getHSVRanges();

        if (params.currentColorMode == DetectionParameters.ColorMode.RED) {
            // Red requires two ranges due to HSV wraparound
            Mat threshold1 = new Mat();
            Mat threshold2 = new Mat();

            Core.inRange(hsvMat, ranges[0], ranges[1], threshold1);
            Core.inRange(hsvMat, ranges[2], ranges[3], threshold2);
            Core.add(threshold1, threshold2, thresholdMat);

            threshold1.release();
            threshold2.release();
        } else {
            // Single range for blue and yellow
            Core.inRange(hsvMat, ranges[0], ranges[1], thresholdMat);
        }
    }

    /**
     * Apply morphological operations (erosion and dilation)
     */
    private static void applyMorphologicalOperations(Mat thresholdMat, Mat morphMat, DetectionParameters params) {
        // Create kernels
        Mat erodeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(params.kernelSize, params.kernelSize));
        Mat dilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(params.kernelSize, params.kernelSize));

        try {
            // Copy threshold to morph matrix
            thresholdMat.copyTo(morphMat);

            // Apply erosion to remove noise
            for (int i = 0; i < params.erodeIterations; i++) {
                Imgproc.erode(morphMat, morphMat, erodeKernel);
            }

            // Apply dilation to fill gaps
            for (int i = 0; i < params.dilateIterations; i++) {
                Imgproc.dilate(morphMat, morphMat, dilateKernel);
            }
        } finally {
            erodeKernel.release();
            dilateKernel.release();
        }
    }

    /**
     * Process a single contour to extract sample pose
     * Uses proper pinhole camera model matching the original Python implementation
     */
    private static SamplePose processContour(MatOfPoint contour, Size imageSize, DetectionParameters params) {
        // Get rotated rectangle
        RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

        // Extract center, size, and angle
        Point center = rotatedRect.center;
        Size size = rotatedRect.size;
        double angle = rotatedRect.angle;

        // Determine longer and shorter dimensions
        double longerDim = Math.max(size.width, size.height);
        double shorterDim = Math.min(size.width, size.height);

        // Adjust angle based on orientation (matching Python logic)
        if (size.width < size.height) {
            angle += 90;
        }

        // Normalize angle to [0, 360) range
        angle = ((angle % 360) + 360) % 360;

        // Calculate aspect ratio and check if it matches expected sample
        double aspectRatio = longerDim / shorterDim;
        double expectedAspectRatio = params.getExpectedAspectRatio();
        boolean aspectMatch = Math.abs(aspectRatio - expectedAspectRatio) < params.aspectRatioTolerance;

        // Calculate distance using pinhole camera model: distance = (real_size * focal_length) / pixel_size
        // Use the longer dimension as it's more stable for distance calculation
        double distance = DetectionParameters.calculateDistanceFromSize(DetectionParameters.SAMPLE_LENGTH, longerDim);

        // Convert pixel coordinates to real-world X,Y coordinates using pinhole camera model
        double[] realWorldCoords = DetectionParameters.pixelToRealWorldCoordinates(
                center.x, center.y, imageSize.width, imageSize.height, distance);
        double xInches = realWorldCoords[0];
        double yInches = realWorldCoords[1];

        // Calculate physical dimensions for validation using the calculated distance
        double[] physicalSize = params.calculatePhysicalSize(size.width, size.height, distance);
        boolean sizeMatch = params.isSampleSizeMatch(physicalSize[0], physicalSize[1]);

        // Calculate confidence based on multiple factors
        double confidence = calculateConfidence(aspectMatch, sizeMatch, longerDim, shorterDim,
                Imgproc.contourArea(contour), params);

        // Apply pose adjustments (calibration)
        double[] adjustedPose = adjustPose(xInches, yInches, angle);

        // Create and return sample pose
        boolean detected = confidence >= params.minConfidence;
        return new SamplePose(adjustedPose[0], adjustedPose[1], adjustedPose[2], detected, 0.0, confidence);
    }

    /**
     * Calculate detection confidence based on multiple factors
     */
    private static double calculateConfidence(boolean aspectMatch, boolean sizeMatch,
                                              double longerDim, double shorterDim, double area,
                                              DetectionParameters params) {
        double confidence = 0.0;

        // Base confidence from contour area (normalized to reasonable range)
        double normalizedArea = Math.min(area / 10000.0, 1.0);
        confidence += normalizedArea * 0.2;

        // Confidence from aspect ratio match
        if (aspectMatch) {
            confidence += 0.3;
        } else {
            // Partial credit for close aspect ratios
            double aspectRatio = longerDim / shorterDim;
            double expectedAspectRatio = params.getExpectedAspectRatio();
            double aspectError = Math.abs(aspectRatio - expectedAspectRatio) / expectedAspectRatio;
            confidence += Math.max(0, 0.3 * (1.0 - aspectError * 2));
        }

        // Confidence from size match
        if (sizeMatch) {
            confidence += 0.3;
        }

        // Confidence from contour size (prefer medium-sized contours)
        double sizeScore = Math.min(longerDim / 100.0, 1.0) * Math.min(100.0 / Math.max(longerDim, 1.0), 1.0);
        confidence += sizeScore * 0.2;

        return Math.min(confidence, 1.0);
    }

    /**
     * Apply pose adjustments (calibration factors)
     * This method can be customized based on camera calibration data
     */
    private static double[] adjustPose(double x, double y, double angle) {
        // Apply calibration factors if available
        // For now, return the original values
        // In a real implementation, you would apply calibration matrices here

        double realX = x;  // Could apply: realX = calibrationMatrix[0][0] * x + calibrationMatrix[0][1] * y + calibrationMatrix[0][2]
        double realY = y;  // Could apply: realY = calibrationMatrix[1][0] * x + calibrationMatrix[1][1] * y + calibrationMatrix[1][2]
        double realAngle = angle;  // Could apply angle calibration

        // Normalize angle to [0, 360) range
        realAngle = ((realAngle % 360) + 360) % 360;

        return new double[]{realX, realY, realAngle};
    }

    /**
     * Estimate physical size from pixel dimensions using camera parameters
     * This matches the original Python implementation
     */
    public static double[] estimatePhysicalSize(double pixelWidth, double pixelHeight,
                                                double distanceInches, DetectionParameters params) {
        return params.calculatePhysicalSize(pixelWidth, pixelHeight, distanceInches);
    }

    /**
     * Calculate distance to object using known object size and pinhole camera model
     * Formula: distance = (real_size * focal_length) / pixel_size
     * This matches the original Python implementation
     */
    public static double calculateDistance(double pixelSize, double realSizeInches) {
        return DetectionParameters.calculateDistanceFromSize(realSizeInches, pixelSize);
    }

    /**
     * Convert pixel coordinates to real-world coordinates using pinhole camera model
     * This matches the original Python implementation
     */
    public static double[] pixelToRealWorld(Point pixelPoint, Size imageSize, double distance) {
        return DetectionParameters.pixelToRealWorldCoordinates(
                pixelPoint.x, pixelPoint.y, imageSize.width, imageSize.height, distance);
    }

    /**
     * Convert pixel coordinates to 3D real-world coordinates
     * Returns [x, y, z] where z is the vertical offset
     */
    public static Point3 pixelToRealWorld3D(Point pixelPoint, Size imageSize, double distance) {
        double imgCenterX = imageSize.width / 2.0;
        double imgCenterY = imageSize.height / 2.0;

        double pixelFromCenterX = pixelPoint.x - imgCenterX;
        double pixelFromCenterY = imgCenterY - pixelPoint.y;

        double angleHorizontal = Math.atan2(pixelFromCenterX, DetectionParameters.FOCAL_LENGTH);
        double angleVertical = Math.atan2(pixelFromCenterY, DetectionParameters.FOCAL_LENGTH);

        double x = distance * Math.sin(angleHorizontal);
        double y = distance * Math.cos(angleHorizontal);
        double z = distance * Math.sin(angleVertical);

        return new Point3(x, y, z);
    }
}
