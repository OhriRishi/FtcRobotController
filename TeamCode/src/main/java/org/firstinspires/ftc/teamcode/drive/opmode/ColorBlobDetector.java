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

import java.util.List;

/**
 * ColorBlobDetector - A class that provides real-world distance and orientation detection
 * using color blob detection.
 * 
 * This class focuses exclusively on color-based object detection without AprilTags,
 * providing methods to detect the distance and angle to colored objects from the
 * center of the robot.
 * 
 * The class accounts for the camera's position and orientation relative to the robot's
 * center to provide accurate measurements from the robot's perspective, not just the
 * camera's perspective.
 */
public class ColorBlobDetector {
    
    // Vision processing objects
    private ColorBlobLocatorProcessor colorProcessor; // Processes camera images to detect colored objects
    private VisionPortal visionPortal;               // Manages camera access and image processing pipeline
    
    // Camera position and calibration values
    // These values define where the camera is mounted on the robot
    private double cameraHeightInches;      // Height of camera from ground
    private double cameraForwardOffset;     // Distance forward from robot center
    private double cameraHorizontalOffset;  // Distance right (positive) or left (negative) from robot center
    private double cameraHorizontalFovDegrees; // Horizontal field of view
    private double cameraVerticalFovDegrees;   // Vertical field of view
    
    // Target colors to detect
    private static final ColorRange RED_TARGET = ColorRange.RED;
    private static final ColorRange BLUE_TARGET = ColorRange.BLUE;
    private static final ColorRange GREEN_TARGET = ColorRange.GREEN;
    private static final ColorRange YELLOW_TARGET = ColorRange.YELLOW;
    
    // Current color mode
    private ColorRange currentColorTarget;
    
    // Robot hardware reference
    private RobotHardware robotHardware;
    
    // Telemetry for debugging and hardware map for camera access
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    
    /**
     * Constructor for the ColorBlobDetector
     * 
     * @param hardwareMap The hardware map from the OpMode
     * @param telemetry The telemetry object for data output
     * @param initialColorTarget The initial color to detect (use ColorRange constants)
     * @param cameraHeightInches Height of camera from ground in inches
     * @param cameraForwardOffset Distance forward from robot center in inches
     * @param cameraHorizontalOffset Distance right (positive) or left (negative) from robot center in inches
     * @param cameraHorizontalFovDegrees Horizontal field of view in degrees
     * @param cameraVerticalFovDegrees Vertical field of view in degrees
     */
    public ColorBlobDetector(HardwareMap hardwareMap, Telemetry telemetry,
                            ColorRange initialColorTarget, double cameraHeightInches, 
                            double cameraForwardOffset, double cameraHorizontalOffset,
                            double cameraHorizontalFovDegrees, double cameraVerticalFovDegrees) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.currentColorTarget = initialColorTarget;
        
        // Store camera calibration values
        this.cameraHeightInches = cameraHeightInches;
        this.cameraForwardOffset = cameraForwardOffset;
        this.cameraHorizontalOffset = cameraHorizontalOffset;
        this.cameraHorizontalFovDegrees = cameraHorizontalFovDegrees;
        this.cameraVerticalFovDegrees = cameraVerticalFovDegrees;
        
        // Initialize the vision system
        initVision(hardwareMap);
    }
    
    /**
     * Constructor with default color target (RED)
     */
    public ColorBlobDetector(HardwareMap hardwareMap, Telemetry telemetry, RobotHardware robotHardware,
                            double cameraHeightInches, double cameraForwardOffset, double cameraHorizontalOffset,
                            double cameraHorizontalFovDegrees, double cameraVerticalFovDegrees) {
        this(hardwareMap, telemetry, ColorRange.RED, cameraHeightInches,
             cameraForwardOffset, cameraHorizontalOffset, cameraHorizontalFovDegrees, cameraVerticalFovDegrees);
    }
    
    /**
     * Initialize the vision processing system
     * 
     * This method sets up the color blob processor and vision portal with the appropriate
     * configuration for detecting colored objects and calculating real-world measurements.
     */
    private void initVision(HardwareMap hardwareMap) {
        // Create the color blob processor
        // This processor detects objects of a specific color
        colorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(currentColorTarget) // Start with the current target color
                .setRoi(ImageRegion.entireFrame())      // Search the entire camera frame
                .setDrawContours(true)                  // Draw outlines around detected color blobs
                .setBlurSize(5)                         // Apply blur to smooth color transitions
                .setErodeSize(2)                        // Erode to remove small noise
                .setDilateSize(2)                       // Dilate to fill small holes
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY) // Only detect outer contours
                .build();
        
        // Create the vision portal builder
        VisionPortal.Builder builder = new VisionPortal.Builder();
        
        // Configure the vision portal with camera settings
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); // Use the configured webcam
        builder.setCameraResolution(new Size(640, 480)); // Standard resolution (balance of speed vs. detail)
        builder.enableLiveView(true);             // Show camera feed on Driver Station
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); // Compressed format to save bandwidth
        builder.addProcessor(colorProcessor);     // Add our color processor to the vision pipeline
        
        // Build and initialize the vision portal
        visionPortal = builder.build();
    }
    
    /**
     * Get the current detected color blobs
     * 
     * @return List of detected color blobs
     */
    public List<ColorBlobLocatorProcessor.Blob> getDetectedBlobs() {
        List<ColorBlobLocatorProcessor.Blob> blobs = colorProcessor.getBlobs();
        
        // Filter blobs by size to remove noise
        // This removes very small detections that are likely false positives
        ColorBlobLocatorProcessor.Util.filterByArea(500, Integer.MAX_VALUE, blobs);
        
        return blobs;
    }
    
    /**
     * Get the number of color blobs currently detected
     * 
     * @return The number of detected color blobs
     */
    public int getBlobCount() {
        return getDetectedBlobs().size();
    }
    
    /**
     * Get the largest detected color blob
     * 
     * @return The largest color blob, or null if none are detected
     */
    public ColorBlobLocatorProcessor.Blob getLargestBlob() {
        List<ColorBlobLocatorProcessor.Blob> blobs = getDetectedBlobs();
        
        if (blobs.isEmpty()) {
            return null;
        }
        
        // The blobs are already sorted by size (largest first)
        return blobs.get(0);
    }
    
    /**
     * Calculate the real-world distance and angle to a color blob from the robot's center
     * 
     * This method converts pixel coordinates and blob size to real-world
     * distance and angle measurements, accounting for the camera's position
     * relative to the robot's center.
     * 
     * @param blob The detected color blob
     * @return Array with [distance_inches, angle_degrees, height_inches, orientation_degrees]
     *         distance_inches: straight-line distance from robot center to object
     *         angle_degrees: angle from robot's forward direction to object (positive = right)
     *         height_inches: estimated height of the object from the ground
     *         orientation_degrees: orientation angle of the rectangular object relative to horizontal
     */
    public double[] calculateDistanceAndAngle(ColorBlobLocatorProcessor.Blob blob) {
        // Get the center of the blob in pixel coordinates
        double centerX = blob.getBoxFit().center.x;
        double centerY = blob.getBoxFit().center.y;
        
        // Calculate horizontal angle based on camera FOV and pixel position
        // This converts the x-coordinate to an angle based on the camera's field of view
        double pixelFromCenter = centerX - 320; // Assuming 640x480 resolution
        double cameraAngleHorizontal = (pixelFromCenter / 320.0) * (cameraHorizontalFovDegrees / 2.0);
        
        // Calculate vertical angle based on camera FOV and pixel position
        // This converts the y-coordinate to an angle based on the camera's field of view
        double pixelFromMiddle = 240 - centerY; // Assuming 640x480 resolution
        double cameraAngleVertical = (pixelFromMiddle / 240.0) * (cameraVerticalFovDegrees / 2.0);
        
        // Estimate distance from camera to object based on blob size and vertical angle
        // This is a simplified model that assumes the object size is inversely
        // proportional to the distance (larger objects are closer)
        double blobArea = blob.getContourArea();
        double estimatedSizeConstant = 5000.0; // This would need calibration for specific objects
        
        // Calculate distance from camera to object
        double distanceFromCamera = estimatedSizeConstant / Math.sqrt(blobArea);
        
        // Adjust for vertical angle (objects at an angle appear farther away)
        distanceFromCamera = distanceFromCamera * Math.cos(Math.toRadians(cameraAngleVertical));
        
        // Calculate 3D coordinates of the object relative to the camera
        double xFromCamera = distanceFromCamera * Math.sin(Math.toRadians(cameraAngleHorizontal));
        double yFromCamera = distanceFromCamera * Math.cos(Math.toRadians(cameraAngleHorizontal));
        double zFromCamera = distanceFromCamera * Math.sin(Math.toRadians(cameraAngleVertical));
        
        // Translate to robot-centered coordinates
        // Account for camera position relative to robot center
        double xFromRobot = xFromCamera + cameraHorizontalOffset;
        double yFromRobot = yFromCamera + cameraForwardOffset;
        double zFromRobot = zFromCamera + cameraHeightInches;
        
        // Calculate final distance and angle from robot center
        double distanceFromRobot = Math.sqrt(xFromRobot*xFromRobot + yFromRobot*yFromRobot);
        double angleFromRobot = Math.toDegrees(Math.atan2(xFromRobot, yFromRobot));
        
        // Calculate the orientation angle of the rectangular object
        // This uses the rotated rectangle fit from the blob detection
        double orientationAngle = calculateOrientationAngle(blob);
        
        // Return distance, angle, height, and orientation
        return new double[] { yFromRobot, angleFromRobot, zFromRobot, orientationAngle, xFromRobot, distanceFromRobot };
    }
    
    /**
     * Calculate the orientation angle of a rectangular object
     * 
     * This method determines the angle of the rectangle relative to horizontal.
     * It uses the rotated rectangle fit from the blob detection to find the
     * orientation of the rectangular object in the image.
     * 
     * @param blob The detected color blob
     * @return The orientation angle in degrees (-90 to 90)
     */
    private double calculateOrientationAngle(ColorBlobLocatorProcessor.Blob blob) {
        // Get the rotated rectangle fit from the blob
        // This provides the best-fit rectangle for the blob, including its orientation
        RotatedRect rotatedRect = blob.getBoxFit();
        
        // Get the angle of the rotated rectangle
        // In OpenCV, the angle is in the range [-90, 0) for angles clockwise from horizontal
        // and in the range [0, 90) for angles counterclockwise from horizontal
        double angle = rotatedRect.angle;
        
        // Get the width and height of the rotated rectangle
        double width = rotatedRect.size.width;
        double height = rotatedRect.size.height;
        
        // Adjust the angle based on the rectangle's dimensions
        // OpenCV's angle is based on the longer side of the rectangle
        // We want the angle to be relative to the horizontal axis
        if (width < height) {
            // If width < height, the angle is relative to the vertical axis
            // We need to adjust it to be relative to the horizontal axis
            angle = angle - 90;
        }
        
        // Normalize the angle to the range [-90, 90]
        // This makes the angle easier to work with
        if (angle < -90) angle += 180;
        if (angle > 90) angle -= 180;
        
        // Invert the angle to match the expected coordinate system
        // In image coordinates, y increases downward, which is opposite to the standard
        // mathematical convention where y increases upward
        angle = -angle;
        
        // Apply perspective correction based on the camera's view angle
        // This adjusts for the fact that objects at different positions in the image
        // have different apparent orientations due to perspective
        // This is a simplified correction and may need refinement for your specific setup
        double centerX = blob.getBoxFit().center.x;
        double centerY = blob.getBoxFit().center.y;
        double pixelFromCenter = centerX - 320; // Assuming 640x480 resolution
        double pixelFromMiddle = 240 - centerY; // Assuming 640x480 resolution
        
        // Calculate the camera's view angles to the object
        double cameraAngleHorizontal = (pixelFromCenter / 320.0) * (cameraHorizontalFovDegrees / 2.0);
        double cameraAngleVertical = (pixelFromMiddle / 240.0) * (cameraVerticalFovDegrees / 2.0);
        
        // Apply a correction factor based on the object's position in the image
        // This is a simplified model and may need calibration for your specific setup
        double horizontalCorrection = cameraAngleHorizontal * 0.2; // Adjust this factor as needed
        double verticalCorrection = cameraAngleVertical * 0.2;     // Adjust this factor as needed
        
        // Apply the corrections
        angle = angle + horizontalCorrection + verticalCorrection;
        
        // Return the final orientation angle
        return angle;
    }
    
    /**
     * Set the target color to detect
     * 
     * @param colorTarget The new color target (use ColorRange constants)
     */
    public void setTargetColor(ColorRange colorTarget) {
        if (colorTarget != currentColorTarget) {
            currentColorTarget = colorTarget;
            updateColorProcessor();
        }
    }
    
    /**
     * Get the current target color
     * 
     * @return The current color target
     */
    public ColorRange getCurrentColorTarget() {
        return currentColorTarget;
    }
    
    /**
     * Update the color processor with the current target color
     * This method recreates the entire vision pipeline with the new color target
     */
    private void updateColorProcessor() {
        // The most compatible approach is to close the existing vision portal
        // and create a new one with the updated color target
        if (visionPortal != null) {
            // Close the current vision portal to release camera resources
            visionPortal.close();
            
            // Wait a moment for resources to be released
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                // Ignore interruption
            }
        }
        
        // Create a new color processor with the current target color
        colorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(currentColorTarget) // Use the current target color
                .setRoi(ImageRegion.entireFrame())      // Search the entire camera frame
                .setDrawContours(true)                  // Draw outlines around detected color blobs
                .setBlurSize(5)                         // Apply blur to smooth color transitions
                .setErodeSize(2)                        // Erode to remove small noise
                .setDilateSize(2)                       // Dilate to fill small holes
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY) // Only detect outer contours
                .build();
        
        // Create a new vision portal with the updated processor
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(colorProcessor);
        
        // Build and initialize the new vision portal
        visionPortal = builder.build();
        
        telemetry.addData("Color Target", "Updated to " + getColorName(currentColorTarget));
    }
    
    /**
     * Add telemetry data about the current detections
     */
    public void addTelemetry() {
        List<ColorBlobLocatorProcessor.Blob> blobs = getDetectedBlobs();
        
        telemetry.addData("Color Target", getColorName(currentColorTarget));
        telemetry.addData("# Objects Detected", blobs.size());
        
        // Display information about each detected blob
        for (int i = 0; i < blobs.size(); i++) {
            ColorBlobLocatorProcessor.Blob blob = blobs.get(i);
            
            // Calculate real-world distance and angle to the blob
            double[] measurements = calculateDistanceAndAngle(blob);
            double distance = measurements[0];
            double angle = measurements[1];
            double height = measurements[2];
            double orientation = measurements[3];
            
            telemetry.addLine(String.format("\n--- Object %d ---", i+1));
            telemetry.addData("Size", String.format("%.2f sq in", blob.getContourArea() / 100.0));
            telemetry.addData("Distance from Robot", String.format("%.2f inches", distance));
            telemetry.addData("Angle from Robot", String.format("%.2f degrees", angle));
            telemetry.addData("Height", String.format("%.2f inches", height));
            telemetry.addData("Orientation", String.format("%.2f degrees", orientation));
            
            // Provide navigation guidance for the largest blob
            if (i == 0) {
                telemetry.addLine("\n--- Navigation Guidance ---");
                
                if (Math.abs(angle) > 5.0) {
                    // Need to turn to align with the object
                    String turnDirection = (angle > 0) ? "RIGHT" : "LEFT";
                    telemetry.addData("Action", String.format("Turn %s to align", turnDirection));
                } else if (distance > 12.0) {
                    // Need to move forward to get closer
                    telemetry.addData("Action", "Move FORWARD");
                } else if (distance < 6.0) {
                    // Too close, need to back up
                    telemetry.addData("Action", "Move BACKWARD");
                } else {
                    // In the optimal position
                    telemetry.addData("Action", "ALIGNED at optimal distance");
                }
            }
        }
        
        // If no objects detected, provide feedback
        if (blobs.isEmpty()) {
            telemetry.addLine("No objects detected. Adjust camera or change target color.");
        }
    }
    
    /**
     * Get a human-readable name for a color range
     * 
     * @param colorRange The color range
     * @return The color name
     */
    private String getColorName(ColorRange colorRange) {
        if (colorRange == RED_TARGET) return "RED";
        if (colorRange == BLUE_TARGET) return "BLUE";
        if (colorRange == GREEN_TARGET) return "GREEN";
        if (colorRange == YELLOW_TARGET) return "YELLOW";
        return "CUSTOM";
    }
    
    /**
     * Pause camera streaming
     */
    public void pauseStreaming() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }
    
    /**
     * Resume camera streaming
     */
    public void resumeStreaming() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }
    
    /**
     * Close the vision portal when done
     * 
     * This method should be called when the OpMode is stopping to release camera resources.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    
    /**
     * Get predefined color targets
     */
    public static ColorRange getRedTarget() { return RED_TARGET; }
    public static ColorRange getBlueTarget() { return BLUE_TARGET; }
    public static ColorRange getGreenTarget() { return GREEN_TARGET; }
    public static ColorRange getYellowTarget() { return YELLOW_TARGET; }
}
