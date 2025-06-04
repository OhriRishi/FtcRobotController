package org.firstinspires.ftc.teamcode.drive.opmode;

import org.opencv.core.Scalar;

/**
 * Configuration parameters for color detection
 * Contains all tunable parameters for the sample detection system
 */
public class DetectionParameters {

    // Camera parameters (Logitech C920)
    public static final double CAMERA_FOV_HORIZONTAL = 78.0;  // Degrees
    public static final double CAMERA_FOV_VERTICAL = 43.0;    // Degrees
    public static final int CAMERA_RESOLUTION_WIDTH = 1280;   // Pixels
    public static final int CAMERA_RESOLUTION_HEIGHT = 720;   // Pixels

    // Sample physical dimensions in inches
    public static final double SAMPLE_LENGTH = 3.5;  // inches
    public static final double SAMPLE_WIDTH = 1.5;   // inches
    public static final double SAMPLE_HEIGHT = 1.5;  // inches

    // Focal length for distance calculations (approximate)
    public static final double FOCAL_LENGTH = 1200;  // pixels

    // Color detection modes
    public enum ColorMode {
        RED, BLUE, YELLOW
    }

    // HSV color ranges for detection
    // Red has two ranges due to HSV wraparound
    public static final Scalar LOW_HSV_RED_1 = new Scalar(0, 100, 100);
    public static final Scalar HIGH_HSV_RED_1 = new Scalar(10, 255, 255);
    public static final Scalar LOW_HSV_RED_2 = new Scalar(160, 100, 100);
    public static final Scalar HIGH_HSV_RED_2 = new Scalar(180, 255, 255);

    // Blue range
    public static final Scalar LOW_HSV_BLUE = new Scalar(100, 100, 100);
    public static final Scalar HIGH_HSV_BLUE = new Scalar(130, 255, 255);

    // Yellow range
    public static final Scalar LOW_HSV_YELLOW = new Scalar(20, 100, 100);
    public static final Scalar HIGH_HSV_YELLOW = new Scalar(40, 255, 255);

    // Morphological operation parameters
    public int erodeIterations = 3;
    public int dilateIterations = 1;
    public int kernelSize = 5;

    // Edge detection parameters
    public boolean useEdgeDetection = false;
    public double cannyThreshold1 = 20.0;
    public double cannyThreshold2 = 50.0;
    public double claheClipLimit = 2.0;
    public int claheTileSize = 8;

    // Contour filtering parameters
    public double minContourArea = 500.0;
    public double maxContourArea = 50000.0;
    public double aspectRatioTolerance = 0.3;
    public double edgeDensityThreshold = 0.05;

    // Detection confidence parameters
    public double minConfidence = 0.5;
    public double sizeMatchTolerance = 0.5;  // inches

    // Current detection mode
    public ColorMode currentColorMode = ColorMode.RED;

    // Adaptive thresholding parameters
    public boolean useAdaptiveThresholding = true;
    public double brightnessScaleFactor = 1.0;
    public double referenceBrightness = 128.0;

    /**
     * Get the expected aspect ratio for samples
     */
    public double getExpectedAspectRatio() {
        return SAMPLE_LENGTH / SAMPLE_WIDTH;
    }

    /**
     * Get HSV ranges for the current color mode
     */
    public Scalar[] getHSVRanges() {
        switch (currentColorMode) {
            case RED:
                return new Scalar[]{LOW_HSV_RED_1, HIGH_HSV_RED_1, LOW_HSV_RED_2, HIGH_HSV_RED_2};
            case BLUE:
                return new Scalar[]{LOW_HSV_BLUE, HIGH_HSV_BLUE};
            case YELLOW:
                return new Scalar[]{LOW_HSV_YELLOW, HIGH_HSV_YELLOW};
            default:
                return new Scalar[]{LOW_HSV_RED_1, HIGH_HSV_RED_1, LOW_HSV_RED_2, HIGH_HSV_RED_2};
        }
    }

    /**
     * Check if the detected dimensions match expected sample size
     */
    public boolean isSampleSizeMatch(double widthInches, double heightInches) {
        // Sort dimensions to compare with length and width (regardless of orientation)
        double[] dims = {widthInches, heightInches};
        java.util.Arrays.sort(dims);

        double longerDim = dims[1];
        double shorterDim = dims[0];

        // Check if dimensions match expected sample dimensions within tolerance
        boolean lengthMatch = Math.abs(longerDim - SAMPLE_LENGTH) <= sizeMatchTolerance;
        boolean widthMatch = Math.abs(shorterDim - SAMPLE_WIDTH) <= sizeMatchTolerance;

        return lengthMatch && widthMatch;
    }

    /**
     * Calculate physical size from pixel dimensions using pinhole camera model
     * This matches the original Python implementation
     */
    public double[] calculatePhysicalSize(double pixelWidth, double pixelHeight, double distanceInches) {
        // Calculate the field of view in radians
        double fovHorizontalRad = Math.toRadians(CAMERA_FOV_HORIZONTAL);
        double fovVerticalRad = Math.toRadians(CAMERA_FOV_VERTICAL);

        // Calculate physical size using pinhole camera model
        // Formula: physical_size = 2 * distance * tan(fov/2) * (pixel_size / image_size)
        double widthInches = 2 * distanceInches * Math.tan(fovHorizontalRad / 2) *
                (pixelWidth / CAMERA_RESOLUTION_WIDTH);
        double heightInches = 2 * distanceInches * Math.tan(fovVerticalRad / 2) *
                (pixelHeight / CAMERA_RESOLUTION_HEIGHT);

        return new double[]{widthInches, heightInches};
    }

    /**
     * Calculate distance using pinhole camera model
     * Formula: distance = (real_size * focal_length) / pixel_size
     */
    public static double calculateDistanceFromSize(double realSizeInches, double pixelSize) {
        return (realSizeInches * FOCAL_LENGTH) / pixelSize;
    }

    /**
     * Convert pixel coordinates to real-world X,Y coordinates
     * This matches the original Python implementation
     */
    public static double[] pixelToRealWorldCoordinates(double centerX, double centerY,
                                                       double imageWidth, double imageHeight,
                                                       double distanceInches) {
        // Calculate offset from image center in pixels
        double imgCenterX = imageWidth / 2.0;
        double imgCenterY = imageHeight / 2.0;
        double pixelFromCenterX = centerX - imgCenterX;
        double pixelFromCenterY = imgCenterY - centerY;  // Invert Y (image coordinates increase downward)

        // Convert pixel offset to angle (in radians)
        double angleHorizontal = Math.atan2(pixelFromCenterX, FOCAL_LENGTH);
        double angleVertical = Math.atan2(pixelFromCenterY, FOCAL_LENGTH);

        // Calculate X and Y coordinates using distance and angles
        // X is horizontal offset (positive to the right)
        // Y is forward distance (positive forward)
        double xInches = distanceInches * Math.sin(angleHorizontal);
        double yInches = distanceInches * Math.cos(angleHorizontal);

        return new double[]{xInches, yInches};
    }
}
