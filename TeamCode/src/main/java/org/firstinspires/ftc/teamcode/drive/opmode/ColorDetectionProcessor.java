package org.firstinspires.ftc.teamcode.drive.opmode;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Vision processor for detecting colored samples using OpenCV
 * Implements the FTC VisionProcessor interface for use with VisionPortal
 */
public class ColorDetectionProcessor implements VisionProcessor {

    private DetectionParameters params;
    private Telemetry telemetry;
    private List<SamplePose> detectedSamples;
    private Mat workingMat = new Mat();
    private Mat hsvMat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat morphMat = new Mat();
    private Mat edgesMat = new Mat();
    private Mat hierarchy = new Mat();

    // Morphological operation kernels
    private Mat erodeKernel;
    private Mat dilateKernel;

    // Processing statistics
    private long lastProcessTime = 0;
    private int frameCount = 0;

    /**
     * Constructor
     */
    public ColorDetectionProcessor(org.firstinspires.ftc.teamcode.drive.opmode.DetectionParameters parameters, Telemetry telemetry) {
        this.params = parameters;
        this.telemetry = telemetry;
        this.detectedSamples = new ArrayList<>();

        // Initialize morphological kernels
        this.erodeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(params.kernelSize, params.kernelSize));
        this.dilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                new Size(params.kernelSize, params.kernelSize));
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        telemetry.addLine("ColorDetectionProcessor initialized");
        telemetry.addData("Resolution", width + "x" + height);
        telemetry.update();
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        long startTime = System.currentTimeMillis();
        frameCount++;

        // Clear previous detections
        detectedSamples.clear();

        // Copy input frame to working matrix
        frame.copyTo(workingMat);

        // Step 1: Convert to HSV color space
        Imgproc.cvtColor(workingMat, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Step 2: Apply color thresholding
        applyColorThresholding();

        // Step 3: Apply morphological operations
        applyMorphologicalOperations();

        // Step 4: Find and process contours
        findAndProcessContours();

        // Step 5: Apply edge detection if enabled
        if (params.useEdgeDetection) {
            applyEdgeDetection();
        }

        // Update processing statistics
        lastProcessTime = System.currentTimeMillis() - startTime;

        // Update telemetry
        updateTelemetry();

        return null;
    }


    /**
     * Apply color thresholding based on current color mode
     */
    private void applyColorThresholding() {
        Scalar[] ranges = params.getHSVRanges();

        if (params.currentColorMode == org.firstinspires.ftc.teamcode.drive.opmode.DetectionParameters.ColorMode.RED) {
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
    private void applyMorphologicalOperations() {
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
    }

    /**
     * Find contours and process them to detect samples
     */
    private void findAndProcessContours() {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(morphMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);

            // Filter by area
            if (area >= params.minContourArea && area <= params.maxContourArea) {
                SamplePose pose = processContour(contour, area);
                if (pose != null) {
                    detectedSamples.add(pose);
                }
            }
        }

        // Release contour memory
        for (MatOfPoint contour : contours) {
            contour.release();
        }
    }

    /**
     * Process a single contour to extract sample pose
     * Uses proper pinhole camera model matching the original Python implementation
     */
    private org.firstinspires.ftc.teamcode.drive.opmode.SamplePose processContour(MatOfPoint contour, double area) {
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
        double distance = org.firstinspires.ftc.teamcode.drive.opmode.DetectionParameters.calculateDistanceFromSize(org.firstinspires.ftc.teamcode.drive.opmode.DetectionParameters.SAMPLE_LENGTH, longerDim);

        // Convert pixel coordinates to real-world X,Y coordinates using pinhole camera model
        double[] realWorldCoords = org.firstinspires.ftc.teamcode.drive.opmode.DetectionParameters.pixelToRealWorldCoordinates(
                center.x, center.y, workingMat.width(), workingMat.height(), distance);
        double xInches = realWorldCoords[0];
        double yInches = realWorldCoords[1];

        // Calculate physical dimensions for validation using the calculated distance
        double[] physicalSize = params.calculatePhysicalSize(size.width, size.height, distance);
        boolean sizeMatch = params.isSampleSizeMatch(physicalSize[0], physicalSize[1]);

        // Calculate confidence based on multiple factors
        double confidence = calculateConfidence(aspectMatch, sizeMatch, area, longerDim);

        // Create and return sample pose
        boolean detected = confidence >= params.minConfidence;
        return new org.firstinspires.ftc.teamcode.drive.opmode.SamplePose(xInches, yInches, angle, detected, 0.0, confidence);
    }

    /**
     * Calculate detection confidence based on multiple factors
     */
    private double calculateConfidence(boolean aspectMatch, boolean sizeMatch, double area, double longerDim) {
        double confidence = 0.0;

        // Base confidence from area (normalized)
        double normalizedArea = Math.min(area / 10000.0, 1.0);
        confidence += normalizedArea * 0.3;

        // Aspect ratio match
        if (aspectMatch) confidence += 0.4;

        // Size match
        if (sizeMatch) confidence += 0.3;

        return Math.min(confidence, 1.0);
    }

    /**
     * Apply edge detection (placeholder for future implementation)
     */
    private void applyEdgeDetection() {
        // Convert to grayscale
        Mat gray = new Mat();
        Imgproc.cvtColor(workingMat, gray, Imgproc.COLOR_RGB2GRAY);

        // Apply Canny edge detection
        Imgproc.Canny(gray, edgesMat, params.cannyThreshold1, params.cannyThreshold2);

        gray.release();
    }

    /**
     * Update telemetry with detection results
     */
    private void updateTelemetry() {
        telemetry.addData("Color Mode", params.currentColorMode);
        telemetry.addData("Samples Detected", detectedSamples.size());
        telemetry.addData("Process Time (ms)", lastProcessTime);
        telemetry.addData("Frame Count", frameCount);

        for (int i = 0; i < detectedSamples.size(); i++) {
            org.firstinspires.ftc.teamcode.drive.opmode.SamplePose pose = detectedSamples.get(i);
            telemetry.addData("Sample " + (i + 1), ((org.firstinspires.ftc.teamcode.drive.opmode.SamplePose) pose).toString());
        }
    }

    /**
     * Get the list of detected samples
     */
    public List<org.firstinspires.ftc.teamcode.drive.opmode.SamplePose> getDetectedSamples() {
        return new ArrayList<>(detectedSamples);
    }

    /**
     * Get detection parameters for tuning
     */
    public DetectionParameters getParameters() {
        return params;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Draw detection results on the camera preview
        // This method can be used to overlay detection visualization
    }
}
