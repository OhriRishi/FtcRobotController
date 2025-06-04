package org.firstinspires.ftc.teamcode.drive.opmode;

/**
 * Class to store the pose of a detected sample
 * Represents the position and orientation of a detected colored sample
 */
public class SamplePose {
    public double xInches;          // Horizontal offset from camera center (positive = right)
    public double yInches;          // Forward distance from camera (positive = forward)
    public double angleDegrees;     // Orientation angle in degrees
    public boolean detected;        // Whether the sample was detected with high confidence
    public double edgeDensity;      // Density of edges within the rotated rectangle
    public double confidence;       // Detection confidence score (0.0 to 1.0)

    /**
     * Default constructor
     */
    public SamplePose() {
        this.xInches = 0.0;
        this.yInches = 0.0;
        this.angleDegrees = 0.0;
        this.detected = false;
        this.edgeDensity = 0.0;
        this.confidence = 0.0;
    }

    /**
     * Full constructor
     */
    public SamplePose(double xInches, double yInches, double angleDegrees,
                      boolean detected, double edgeDensity, double confidence) {
        this.xInches = xInches;
        this.yInches = yInches;
        this.angleDegrees = angleDegrees;
        this.detected = detected;
        this.edgeDensity = edgeDensity;
        this.confidence = confidence;
    }

    /**
     * Copy constructor
     */
    public SamplePose(SamplePose other) {
        this.xInches = other.xInches;
        this.yInches = other.yInches;
        this.angleDegrees = other.angleDegrees;
        this.detected = other.detected;
        this.edgeDensity = other.edgeDensity;
        this.confidence = other.confidence;
    }

    /**
     * Get distance from camera center
     */
    public double getDistanceFromCamera() {
        return Math.sqrt(xInches * xInches + yInches * yInches);
    }

    /**
     * Get angle to target from camera center (in degrees)
     */
    public double getAngleToTarget() {
        return Math.toDegrees(Math.atan2(xInches, yInches));
    }

    /**
     * String representation for debugging
     */
    @Override
    public String toString() {
        return String.format("SamplePose{x=%.1f, y=%.1f, angle=%.1f°, detected=%s, confidence=%.2f}",
                xInches, yInches, angleDegrees, detected, confidence);
    }

    /**
     * Check if this pose represents a valid detection
     */
    public boolean isValid() {
        return detected && confidence > 0.5;
    }
}
