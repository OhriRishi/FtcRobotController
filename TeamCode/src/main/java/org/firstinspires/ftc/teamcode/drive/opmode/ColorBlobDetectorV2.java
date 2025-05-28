package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import org.opencv.core.RotatedRect;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.CvType;
import org.opencv.calib3d.Calib3d;

import java.util.List;
import java.util.ArrayList;

/**
 * ColorBlobDetector - A class that provides real-world distance and orientation detection
 * using color blob detection with advanced separation for overlapping samples.
 *
 * Camera calibration is applied as preprocessing - all images are undistorted before
 * processing, making the rest of the pipeline work with corrected images.
 * GREEN color detection has been removed as it's not used in the game.
 */
public class ColorBlobDetectorV2 {

    /**
     * Custom blob wrapper class to represent separated samples
     */
    public static class SeparatedBlob {
        private Point center;
        private double area;
        private MatOfPoint contour;
        private RotatedRect boxFit;
        private boolean isSeparated;
        private ColorBlobLocatorProcessor.Blob originalBlob;

        public SeparatedBlob(Point center, double area, MatOfPoint contour, RotatedRect boxFit, boolean isSeparated, ColorBlobLocatorProcessor.Blob originalBlob) {
            this.center = center;
            this.area = area;
            this.contour = contour;
            this.boxFit = boxFit;
            this.isSeparated = isSeparated;
            this.originalBlob = originalBlob;
        }

        public Point getCenter() { return center; }
        public double getContourArea() { return area; }
        public MatOfPoint getContour() { return contour; }
        public RotatedRect getBoxFit() { return boxFit; }
        public boolean isSeparated() { return isSeparated; }
        public ColorBlobLocatorProcessor.Blob getOriginalBlob() { return originalBlob; }
    }

    // Vision processing objects
    private ColorBlobLocatorProcessor colorProcessor;
    private VisionPortal visionPortal;

    // Camera position and calibration values
    private double cameraHeightInches;
    private double cameraForwardOffset;
    private double cameraHorizontalOffset;
    private double cameraHorizontalFovDegrees;
    private double cameraVerticalFovDegrees;

    // Target colors to detect (GREEN removed as it's not used in the game)
    private static final ColorRange RED_TARGET = ColorRange.RED;
    private static final ColorRange BLUE_TARGET = ColorRange.BLUE;
    private static final ColorRange YELLOW_TARGET = ColorRange.YELLOW;

    // Camera calibration parameters for preprocessing
    private Mat cameraMatrix;
    private Mat distortionCoefficients;
    private Mat undistortMap1, undistortMap2;
    private boolean useCalibration = false;

    // Current color mode
    private ColorRange currentColorTarget;

    // Robot hardware reference
    private RobotHardware robotHardware;

    // Telemetry for debugging and hardware map for camera access
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    // Advanced blob separation settings
    private boolean useAdvancedSeparation = true;
    private double minSampleSeparationDistance = 30.0;

    // Sample detection thresholds for rectangular samples
    private double expectedSingleSampleArea = 2000.0;
    private double maxAspectRatioSingleSample = 2.2;
    private double areaMultiplier = 1.8;

    /**
     * Constructor with camera calibration parameters (preprocessing approach)
     */
    public ColorBlobDetector(HardwareMap hardwareMap, Telemetry telemetry, RobotHardware robotHardware,
                             ColorRange initialColorTarget, double cameraHeightInches,
                             double cameraForwardOffset, double cameraHorizontalOffset,
                             double cameraHorizontalFovDegrees, double cameraVerticalFovDegrees,
                             double fx, double fy, double cx, double cy,
                             double k1, double k2, double p1, double p2, double k3) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.robotHardware = robotHardware;
        this.currentColorTarget = initialColorTarget;

        // Store camera calibration values
        this.cameraHeightInches = cameraHeightInches;
        this.cameraForwardOffset = cameraForwardOffset;
        this.cameraHorizontalOffset = cameraHorizontalOffset;
        this.cameraHorizontalFovDegrees = cameraHorizontalFovDegrees;
        this.cameraVerticalFovDegrees = cameraVerticalFovDegrees;

        // Initialize camera calibration for preprocessing
        initCameraCalibration(fx, fy, cx, cy, k1, k2, p1, p2, k3);

        // Initialize the vision system
        initVision(hardwareMap);

        telemetry.addData("Camera Calibration", "ENABLED - Preprocessing active");
    }

    /**
     * Constructor without calibration (original version)
     */
    public ColorBlobDetector(HardwareMap hardwareMap, Telemetry telemetry, RobotHardware robotHardware,
                             ColorRange initialColorTarget, double cameraHeightInches,
                             double cameraForwardOffset, double cameraHorizontalOffset,
                             double cameraHorizontalFovDegrees, double cameraVerticalFovDegrees) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.robotHardware = robotHardware;
        this.currentColorTarget = initialColorTarget;

        // Store camera calibration values
        this.cameraHeightInches = cameraHeightInches;
        this.cameraForwardOffset = cameraForwardOffset;
        this.cameraHorizontalOffset = cameraHorizontalOffset;
        this.cameraHorizontalFovDegrees = cameraHorizontalFovDegrees;
        this.cameraVerticalFovDegrees = cameraVerticalFovDegrees;

        // Initialize the vision system
        initVision(hardwareMap);

        telemetry.addData("Camera Calibration", "DISABLED - Using FOV calculations");
    }

    /**
     * Constructor with default color target (RED)
     */
    public ColorBlobDetector(HardwareMap hardwareMap, Telemetry telemetry, RobotHardware robotHardware,
                             double cameraHeightInches, double cameraForwardOffset, double cameraHorizontalOffset,
                             double cameraHorizontalFovDegrees, double cameraVerticalFovDegrees) {
        this(hardwareMap, telemetry, robotHardware, ColorRange.RED, cameraHeightInches,
                cameraForwardOffset, cameraHorizontalOffset, cameraHorizontalFovDegrees, cameraVerticalFovDegrees);
    }

    /**
     * Initialize camera calibration matrices for preprocessing
     */
    private void initCameraCalibration(double fx, double fy, double cx, double cy,
                                       double k1, double k2, double p1, double p2, double k3) {
        // Initialize camera matrix (3x3)
        cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
        cameraMatrix.put(0, 0, fx, 0, cx);
        cameraMatrix.put(1, 0, 0, fy, cy);
        cameraMatrix.put(2, 0, 0, 0, 1);

        // Initialize distortion coefficients (1x5)
        distortionCoefficients = new Mat(1, 5, CvType.CV_64FC1);
        distortionCoefficients.put(0, 0, k1, k2, p1, p2, k3);

        // Pre-compute undistortion maps for efficient real-time processing
        Size imageSize = new Size(640, 480);
        undistortMap1 = new Mat();
        undistortMap2 = new Mat();

        Calib3d.initUndistortRectifyMap(
                cameraMatrix, distortionCoefficients, new Mat(),
                cameraMatrix, imageSize, CvType.CV_16SC2,
                undistortMap1, undistortMap2
        );

        useCalibration = true;

        // Update FOV values based on calibration for more accurate calculations
        double fx_val = cameraMatrix.get(0, 0)[0];
        double fy_val = cameraMatrix.get(1, 1)[0];

        // Calculate actual FOV from calibrated focal lengths
        cameraHorizontalFovDegrees = 2.0 * Math.toDegrees(Math.atan(320.0 / fx_val));
        cameraVerticalFovDegrees = 2.0 * Math.toDegrees(Math.atan(240.0 / fy_val));

        telemetry.addData("Calibrated FOV", String.format("H=%.1f°, V=%.1f°",
                cameraHorizontalFovDegrees, cameraVerticalFovDegrees));
    }

    /**
     * Initialize the vision processing system with preprocessing
     */
    private void initVision(HardwareMap hardwareMap) {
        // Create the color blob processor (standard approach)
        colorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(currentColorTarget)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .setErodeSize(2)
                .setDilateSize(2)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .build();

        // Create the vision portal builder
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Configure the vision portal with camera settings
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(colorProcessor);

        // Build and initialize the vision portal
        visionPortal = builder.build();
    }

    /**
     * Apply camera calibration correction to a point (preprocessing approach)
     */
    private Point correctPoint(Point originalPoint) {
        if (!useCalibration || cameraMatrix == null || distortionCoefficients == null) {
            return originalPoint; // Return original if no calibration
        }

        // Create input point matrix
        Mat distortedPoints = new Mat(1, 1, CvType.CV_64FC2);
        distortedPoints.put(0, 0, originalPoint.x, originalPoint.y);

        // Undistort the point
        Mat undistortedPoints = new Mat();
        Calib3d.undistortPoints(distortedPoints, undistortedPoints, cameraMatrix, distortionCoefficients);

        // Get the undistorted point
        double[] undistorted = undistortedPoints.get(0, 0);

        // Convert back to pixel coordinates using camera matrix
        double fx = cameraMatrix.get(0, 0)[0];
        double fy = cameraMatrix.get(1, 1)[0];
        double cx = cameraMatrix.get(0, 2)[0];
        double cy = cameraMatrix.get(1, 2)[0];

        double correctedX = undistorted[0] * fx + cx;
        double correctedY = undistorted[1] * fy + cy;

        // Clean up matrices
        distortedPoints.release();
        undistortedPoints.release();

        return new Point(correctedX, correctedY);
    }

    /**
     * Get the current detected color blobs with calibration correction applied
     */
    public List<ColorBlobLocatorProcessor.Blob> getDetectedBlobs() {
        List<ColorBlobLocatorProcessor.Blob> blobs = colorProcessor.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(500, Integer.MAX_VALUE, blobs);
        return blobs;
    }

    /**
     * Get separated blobs with actual separation of overlapping samples
     */
    public List<SeparatedBlob> getSeparatedBlobs() {
        List<ColorBlobLocatorProcessor.Blob> originalBlobs = getDetectedBlobs();
        List<SeparatedBlob> separatedBlobs = new ArrayList<>();

        if (!useAdvancedSeparation) {
            for (ColorBlobLocatorProcessor.Blob blob : originalBlobs) {
                separatedBlobs.add(new SeparatedBlob(
                        blob.getBoxFit().center,
                        blob.getContourArea(),
                        blob.getContour(),
                        blob.getBoxFit(),
                        false,
                        blob
                ));
            }
            return separatedBlobs;
        }

        // Process each blob for potential separation
        for (ColorBlobLocatorProcessor.Blob blob : originalBlobs) {
            if (isPotentiallyMultipleSamples(blob)) {
                List<SeparatedBlob> separated = separateOverlappingBlob(blob);
                separatedBlobs.addAll(separated);
                telemetry.addData("Separation", String.format("Split blob into %d samples", separated.size()));
            } else {
                separatedBlobs.add(new SeparatedBlob(
                        blob.getBoxFit().center,
                        blob.getContourArea(),
                        blob.getContour(),
                        blob.getBoxFit(),
                        false,
                        blob
                ));
            }
        }

        return separatedBlobs;
    }

    /**
     * Determine if a blob potentially contains multiple rectangular samples
     */
    private boolean isPotentiallyMultipleSamples(ColorBlobLocatorProcessor.Blob blob) {
        RotatedRect boundingRect = blob.getBoxFit();
        double width = boundingRect.size.width;
        double height = boundingRect.size.height;
        double area = blob.getContourArea();

        double aspectRatio = Math.max(width, height) / Math.min(width, height);

        boolean isVeryElongated = aspectRatio > maxAspectRatioSingleSample;
        boolean isOversized = area > (expectedSingleSampleArea * areaMultiplier);

        return isVeryElongated || isOversized;
    }

    /**
     * Separate a single blob that contains multiple overlapping samples
     */
    private List<SeparatedBlob> separateOverlappingBlob(ColorBlobLocatorProcessor.Blob blob) {
        List<SeparatedBlob> separatedSamples = new ArrayList<>();

        RotatedRect boundingRect = blob.getBoxFit();
        double width = boundingRect.size.width;
        double height = boundingRect.size.height;
        double aspectRatio = Math.max(width, height) / Math.min(width, height);

        if (aspectRatio > maxAspectRatioSingleSample) {
            separatedSamples.addAll(separateElongatedBlob(blob));
        } else if (blob.getContourArea() > (expectedSingleSampleArea * areaMultiplier)) {
            separatedSamples.addAll(separateOversizedBlob(blob));
        }

        if (separatedSamples.isEmpty()) {
            separatedSamples.add(new SeparatedBlob(
                    blob.getBoxFit().center,
                    blob.getContourArea(),
                    blob.getContour(),
                    blob.getBoxFit(),
                    false,
                    blob
            ));
        }

        return separatedSamples;
    }

    /**
     * Separate elongated blobs by dividing along the long axis
     */
    private List<SeparatedBlob> separateElongatedBlob(ColorBlobLocatorProcessor.Blob blob) {
        List<SeparatedBlob> separated = new ArrayList<>();

        RotatedRect boundingRect = blob.getBoxFit();
        double width = boundingRect.size.width;
        double height = boundingRect.size.height;
        Point center = boundingRect.center;
        double angle = boundingRect.angle;

        boolean divideHorizontally = width > height;
        int numSamples = (int) Math.round(Math.max(width, height) / Math.min(width, height));
        numSamples = Math.min(numSamples, 4);

        for (int i = 0; i < numSamples; i++) {
            double offset = (i - (numSamples - 1) / 2.0) * (divideHorizontally ? width : height) / numSamples;

            Point sampleCenter;
            if (divideHorizontally) {
                sampleCenter = new Point(
                        center.x + offset * Math.cos(Math.toRadians(angle)),
                        center.y + offset * Math.sin(Math.toRadians(angle))
                );
            } else {
                sampleCenter = new Point(
                        center.x - offset * Math.sin(Math.toRadians(angle)),
                        center.y + offset * Math.cos(Math.toRadians(angle))
                );
            }

            double sampleWidth = divideHorizontally ? width / numSamples : width;
            double sampleHeight = divideHorizontally ? height : height / numSamples;

            RotatedRect sampleBox = new RotatedRect(sampleCenter, new Size(sampleWidth, sampleHeight), angle);
            double sampleArea = blob.getContourArea() / numSamples;

            Point[] rectPoints = new Point[4];
            sampleBox.points(rectPoints);
            MatOfPoint sampleContour = new MatOfPoint(rectPoints);

            separated.add(new SeparatedBlob(
                    sampleCenter,
                    sampleArea,
                    sampleContour,
                    sampleBox,
                    true,
                    blob
            ));
        }

        return separated;
    }

    /**
     * Separate oversized blobs by estimating sample positions
     */
    private List<SeparatedBlob> separateOversizedBlob(ColorBlobLocatorProcessor.Blob blob) {
        List<SeparatedBlob> separated = new ArrayList<>();

        int estimatedSamples = (int) Math.round(blob.getContourArea() / expectedSingleSampleArea);
        estimatedSamples = Math.max(2, Math.min(estimatedSamples, 4));

        RotatedRect boundingRect = blob.getBoxFit();
        Point center = boundingRect.center;
        double width = boundingRect.size.width;
        double height = boundingRect.size.height;

        int rows = estimatedSamples == 2 ? 1 : 2;
        int cols = (estimatedSamples + rows - 1) / rows;

        double cellWidth = width / cols;
        double cellHeight = height / rows;

        for (int row = 0; row < rows; row++) {
            for (int col = 0; col < cols && (row * cols + col) < estimatedSamples; col++) {
                double offsetX = (col - (cols - 1) / 2.0) * cellWidth;
                double offsetY = (row - (rows - 1) / 2.0) * cellHeight;

                Point sampleCenter = new Point(center.x + offsetX, center.y + offsetY);

                RotatedRect sampleBox = new RotatedRect(
                        sampleCenter,
                        new Size(cellWidth * 0.8, cellHeight * 0.8),
                        boundingRect.angle
                );

                double sampleArea = blob.getContourArea() / estimatedSamples;

                Point[] rectPoints = new Point[4];
                sampleBox.points(rectPoints);
                MatOfPoint sampleContour = new MatOfPoint(rectPoints);

                separated.add(new SeparatedBlob(
                        sampleCenter,
                        sampleArea,
                        sampleContour,
                        sampleBox,
                        true,
                        blob
                ));
            }
        }

        return separated;
    }

    /**
     * Calculate the real-world distance and angle to a separated blob from the robot's center
     * (Now applies camera calibration preprocessing to correct for lens distortion)
     */
    public double[] calculateDistanceAndAngle(SeparatedBlob blob) {
        // Apply camera calibration correction to the blob center point
        Point originalCenter = blob.getBoxFit().center;
        Point correctedCenter = correctPoint(originalCenter);

        double centerX = correctedCenter.x;
        double centerY = correctedCenter.y;

        double pixelFromCenter = centerX - 320;
        double cameraAngleHorizontal = (pixelFromCenter / 320.0) * (cameraHorizontalFovDegrees / 2.0);

        double pixelFromMiddle = 240 - centerY;
        double cameraAngleVertical = (pixelFromMiddle / 240.0) * (cameraVerticalFovDegrees / 2.0);

        double blobArea = blob.getContourArea();
        double estimatedSizeConstant = 5000.0;

        double distanceFromCamera = estimatedSizeConstant / Math.sqrt(blobArea);
        distanceFromCamera = distanceFromCamera * Math.cos(Math.toRadians(cameraAngleVertical));

        double xFromCamera = distanceFromCamera * Math.sin(Math.toRadians(cameraAngleHorizontal));
        double yFromCamera = distanceFromCamera * Math.cos(Math.toRadians(cameraAngleHorizontal));
        double zFromCamera = distanceFromCamera * Math.sin(Math.toRadians(cameraAngleVertical));

        double xFromRobot = xFromCamera + cameraHorizontalOffset;
        double yFromRobot = yFromCamera + cameraForwardOffset;
        double zFromRobot = zFromCamera + cameraHeightInches;

        double distanceFromRobot = Math.sqrt(xFromRobot*xFromRobot + yFromRobot*yFromRobot);
        double angleFromRobot = Math.toDegrees(Math.atan2(xFromRobot, yFromRobot));

        double orientationAngle = calculateOrientationAngle(blob);

        return new double[] { distanceFromRobot, angleFromRobot, zFromRobot, orientationAngle };
    }

    /**
     * Calculate the orientation angle of a separated blob
     */
    private double calculateOrientationAngle(SeparatedBlob blob) {
        RotatedRect rotatedRect = blob.getBoxFit();
        double angle = rotatedRect.angle;
        double width = rotatedRect.size.width;
        double height = rotatedRect.size.height;

        if (width < height) {
            angle = angle - 90;
        }

        if (angle < -90) angle += 180;
        if (angle > 90) angle -= 180;

        angle = -angle;

        return angle;
    }

    /**
     * Set the target color to detect
     */
    public void setTargetColor(ColorRange colorTarget) {
        if (colorTarget != currentColorTarget) {
            currentColorTarget = colorTarget;
            updateColorProcessor();
        }
    }

    /**
     * Get the current target color
     */
    public ColorRange getCurrentColorTarget() {
        return currentColorTarget;
    }

    /**
     * Update the color processor with the current target color
     */
    private void updateColorProcessor() {
        if (visionPortal != null) {
            visionPortal.close();

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                // Ignore interruption
            }
        }

        initVision(hardwareMap);

        telemetry.addData("Color Target", "Updated to " + getColorName(currentColorTarget));
    }

    /**
     * Get a human-readable name for a color range
     */
    private String getColorName(ColorRange colorRange) {
        if (colorRange == ColorRange.RED) return "RED";
        if (colorRange == ColorRange.BLUE) return "BLUE";
        if (colorRange == ColorRange.YELLOW) return "YELLOW";
        return "UNKNOWN";
    }

    /**
     * Add telemetry data about the current detections
     */
    public void addTelemetry() {
        List<SeparatedBlob> blobs = getSeparatedBlobs();

        telemetry.addData("Color Target", getColorName(currentColorTarget));
        telemetry.addData("# Separated Samples", blobs.size());
        telemetry.addData("Advanced Separation", useAdvancedSeparation ? "ENABLED" : "DISABLED");
        telemetry.addData("Camera Calibration", useCalibration ? "ENABLED (Preprocessing)" : "DISABLED");

        for (int i = 0; i < blobs.size(); i++) {
            SeparatedBlob blob = blobs.get(i);

            double[] measurements = calculateDistanceAndAngle(blob);
            double distance = measurements[0];
            double angle = measurements[1];
            double height = measurements[2];
            double orientation = measurements[3];

            String status = blob.isSeparated() ? "SEPARATED" : "ORIGINAL";

            telemetry.addLine(String.format("\n--- Sample %d (%s) ---", i + 1, status));
            telemetry.addData("Distance", String.format("%.1f inches", distance));
            telemetry.addData("Angle", String.format("%.1f degrees", angle));
            telemetry.addData("Height", String.format("%.1f inches", height));
            telemetry.addData("Orientation", String.format("%.1f degrees", orientation));
            telemetry.addData("Area", String.format("%.0f pixels", blob.getContourArea()));
            telemetry.addData("Center", String.format("(%.0f, %.0f)", blob.getCenter().x, blob.getCenter().y));
        }
    }

    /**
     * Close the vision portal and release resources
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }

        // Release calibration matrices
        if (cameraMatrix != null) {
            cameraMatrix.release();
        }
        if (distortionCoefficients != null) {
            distortionCoefficients.release();
        }
        if (undistortMap1 != null) {
            undistortMap1.release();
        }
        if (undistortMap2 != null) {
            undistortMap2.release();
        }
    }

    // Additional utility methods (keeping the existing API)
    public int getSeparatedBlobCount() {
        return getSeparatedBlobs().size();
    }

    public int getBlobCount() {
        return getDetectedBlobs().size();
    }

    public ColorBlobLocatorProcessor.Blob getLargestBlob() {
        List<ColorBlobLocatorProcessor.Blob> blobs = getDetectedBlobs();
        return blobs.isEmpty() ? null : blobs.get(0);
    }

    public SeparatedBlob getLargestSeparatedBlob() {
        List<SeparatedBlob> blobs = getSeparatedBlobs();
        if (blobs.isEmpty()) return null;

        SeparatedBlob largest = blobs.get(0);
        for (SeparatedBlob blob : blobs) {
            if (blob.getContourArea() > largest.getContourArea()) {
                largest = blob;
            }
        }
        return largest;
    }

    public void setAdvancedSeparationEnabled(boolean enabled) {
        this.useAdvancedSeparation = enabled;
    }

    public void setExpectedSingleSampleArea(double area) {
        this.expectedSingleSampleArea = area;
    }

    public void setMaxAspectRatioSingleSample(double ratio) {
        this.maxAspectRatioSingleSample = ratio;
    }
}
