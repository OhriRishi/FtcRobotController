package org.firstinspires.ftc.teamcode.drive.opmode;

import java.util.ArrayList;
import java.util.List;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Config
@TeleOp(name = "Limelight Vision OpMode")
public class LimelightOpMode extends LinearOpMode {

    // Color enum
    public enum Color {
        RED, BLUE, YELLOW
    }

    // Location class
    public class Location {
        public double translation;
        public double extension;
        public double rotation;
        public double score;
        public Color color;
        public double rotScore;
        public double distScore;
        public double yScore;
        public double orientationAngle; // Actual orientation angle in degrees

        public Location(double trans, double extension, double rotation, Color color) {
            this.translation = trans;
            this.extension = extension;
            this.rotation = rotation;
            this.color = color;
            this.orientationAngle = 0; // Initialize to 0
        }
    }

    // Constants class
    @Config
    public static class LimelightConstants {
        public static double IDEAL_ASPECT_RATIO = 3.5 / 1.5; // Expected width:height ratio for a properly aligned sample
        public static double IDEAL_Y = 0; // Set to 0 for camera-only testing
        public static int PIPELINE = 2  ;
        public static double LIME_LIGHT_MOUNT_ANGLE = 30; // a1
        public static double LIME_LIGHT_LENS_HEIGHT_INCHES = 6; // h1
        public static double LIME_LIGHT_OFFSET = 0; // h1
        public static double SAMPLE_HEIGHT_INCHES = 0; // h2
        public static double TELESCOPE_OFFSET = 0; // Set to 0 for camera-only testing
        public static double X_WEIGHT = 2;
        public static double Y_WEIGHT = 3;
        public static double ROT_WEIGHT = 20;
    }

    private Limelight3A limelight;
    private ArrayList<Location> locations = new ArrayList<>();
    private Color targetColor = Color.BLUE; // Set your target color here

    @Override
    public void runOpMode() {
        // Initialize hardware
        initLimelight();

        waitForStart();

        while (opModeIsActive()) {
            updateDetections();

            Location best = getBest();
            displayTelemetry(best);

            // Add your robot control logic here based on the best detection

            sleep(50); // Small delay to prevent overwhelming the system
        }

        // Clean up
        if (limelight != null) {
            limelight.stop();
        }
    }

    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(LimelightConstants.PIPELINE);
        limelight.start();
    }

    private void updateDetections() {
        // thanks 20077 :)
        LLResult result = limelight.getLatestResult();
        if (result == null)
            return;
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        // List to hold scored detections
        locations = new ArrayList<>();

        for (LLResultTypes.DetectorResult detection : detections) {
            Color color; // Detected class (color)
            switch (detection.getClassId()) {
                case 0:
                    color = Color.BLUE;
                    break;
                case 1:
                    color = Color.RED;
                    break;
                case 2:
                    color = Color.YELLOW;
                    break;
                default:
                    color = Color.YELLOW;
                    break;
            }

            // Calculate bounding box dimensions
            List<List<Double>> corners = detection.getTargetCorners();
            if (corners == null || corners.size() < 4) {
                continue; // Skip invalid detections
            }

            // Debug: Print corner coordinates (only for first detection to avoid spam)
            if (locations.size() == 0) {
                telemetry.addLine("--- Corner Debug ---");
                for (int i = 0; i < corners.size(); i++) {
                    telemetry.addData("Corner " + i, String.format("(%.2f, %.2f)",
                            corners.get(i).get(0), corners.get(i).get(1)));
                }
            }

            double width = calculateDistance(corners.get(0), corners.get(1)); // Top edge
            double height = calculateDistance(corners.get(1), corners.get(2)); // Side edge

            // Try multiple orientation calculation methods
            double orientationAngle1 = calculateOrientation(corners);
            double orientationAngle2 = calculateOrientationAlternative(corners);

            // Debug orientation methods (only for first detection)
            if (locations.size() == 0) {
                telemetry.addData("Orientation Method 1", String.format("%.1f°", orientationAngle1));
                telemetry.addData("Orientation Method 2", String.format("%.1f°", orientationAngle2));
                telemetry.addData("Width", String.format("%.2f", width));
                telemetry.addData("Height", String.format("%.2f", height));
                telemetry.addData("Aspect Ratio", String.format("%.2f", width/height));
            }

            // Calculate aspect ratio and rotation score
            double aspectRatio = width / height;
            double rotationScore = Math.abs(aspectRatio - LimelightConstants.IDEAL_ASPECT_RATIO);

            // Calculate distance (approximation based on angles)
            double actualYAngle = LimelightConstants.LIME_LIGHT_MOUNT_ANGLE - detection.getTargetYDegrees();
            double yDistance = (LimelightConstants.LIME_LIGHT_LENS_HEIGHT_INCHES - LimelightConstants.SAMPLE_HEIGHT_INCHES)
                    / Math.tan(Math.toRadians(actualYAngle)) + LimelightConstants.TELESCOPE_OFFSET;
            double xDistance = Math.tan(Math.toRadians(detection.getTargetXDegrees())) * yDistance - LimelightConstants.LIME_LIGHT_OFFSET;

            // Create location with orientation angle
            Location loc = new Location(xDistance, yDistance, rotationScore, color);
            loc.orientationAngle = orientationAngle2; // Use the alternative method
            locations.add(loc);
        }
    }

    // Original method
    private double calculateOrientation(List<List<Double>> corners) {
        // Get the top edge of the bounding box (corners 0 to 1)
        double x1 = corners.get(0).get(0);
        double y1 = corners.get(0).get(1);
        double x2 = corners.get(1).get(0);
        double y2 = corners.get(1).get(1);

        // Calculate angle of the top edge relative to horizontal
        double angle = Math.toDegrees(Math.atan2(y2 - y1, x2 - x1));

        // Normalize to -90 to +90 degrees
        if (angle > 90) {
            angle -= 180;
        } else if (angle < -90) {
            angle += 180;
        }

        return angle;
    }

    // Alternative method - try different corner pairs
    private double calculateOrientationAlternative(List<List<Double>> corners) {
        // Find the longest edge to determine orientation
        double maxLength = 0;
        double bestAngle = 0;

        // Check all possible edges
        for (int i = 0; i < 4; i++) {
            int next = (i + 1) % 4;
            double x1 = corners.get(i).get(0);
            double y1 = corners.get(i).get(1);
            double x2 = corners.get(next).get(0);
            double y2 = corners.get(next).get(1);

            double length = Math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));

            if (length > maxLength) {
                maxLength = length;
                double angle = Math.toDegrees(Math.atan2(y2 - y1, x2 - x1));

                // Normalize to -90 to +90 degrees
                if (angle > 90) {
                    angle -= 180;
                } else if (angle < -90) {
                    angle += 180;
                }

                bestAngle = angle;
            }
        }

        return bestAngle;
    }

    private boolean isColor(Color color) {
        return targetColor == color || color == Color.YELLOW;
    }

    private Location getBest() {
        if (locations.size() == 0) {
            return new Location(0, 0, 0, Color.YELLOW);
        }

        locations.sort((a, b) -> Double.compare(b.translation, a.translation));
        for (int i = 0; i < locations.size(); i++) {
            Location current = locations.get(i);
            if (!isColor(current.color)) {
                current.score = Integer.MIN_VALUE;
                continue;
            }
            Location left = new Location(current.translation, 0, 0, Color.YELLOW);
            Location right = new Location(current.translation, 0, 0, Color.YELLOW);
            if (i > 0) {
                left = locations.get(i - 1);
            }
            if (i < locations.size() - 1) {
                right = locations.get(i + 1);
            }

            double leftDist = Math.abs(current.translation - left.translation);
            double rightDist = Math.abs(current.translation - right.translation);
            current.distScore = 0;
            if (!isColor(left.color)) {
                if (leftDist == 0) {
                    leftDist = 0.00001;
                }
                current.distScore -= LimelightConstants.X_WEIGHT / leftDist;
            }
            if (!isColor(right.color)) {
                if (rightDist == 0) {
                    rightDist = 0.00001;
                }
                current.distScore -= LimelightConstants.X_WEIGHT / rightDist;
            }
            current.distScore += LimelightConstants.X_WEIGHT * (leftDist + rightDist);
            current.rotScore = -LimelightConstants.ROT_WEIGHT * current.rotation * current.rotation * current.rotation;
            current.yScore = -LimelightConstants.Y_WEIGHT * Math.abs(current.extension - LimelightConstants.IDEAL_Y);
            current.score = current.distScore + current.rotScore + current.yScore;
        }

        locations.sort((a, b) -> Double.compare(b.score, a.score));

        return locations.get(0);
    }

    private double calculateDistance(List<Double> point1, List<Double> point2) {
        double dx = point1.get(0) - point2.get(0);
        double dy = point1.get(1) - point2.get(1);
        return Math.sqrt(dx * dx + dy * dy);
    }

    private void displayTelemetry(Location best) {
        telemetry.addData("Status", "Running");
        telemetry.addData("Target Color", targetColor.toString());
        telemetry.addData("Samples Detected", locations.size());
        telemetry.addLine("--- Best Target ---");
        telemetry.addData("X Position", String.format("%.2f", best.translation));
        telemetry.addData("Y Position", String.format("%.2f", best.extension));
        telemetry.addData("Orientation", String.format("%.1f°", best.orientationAngle));
        telemetry.addData("Color", best.color.toString());
        telemetry.addData("Rotation Score", String.format("%.2f", best.rotScore));
        telemetry.addData("Distance Score", String.format("%.2f", best.distScore));
        telemetry.addData("Y Score", String.format("%.2f", best.yScore));
        telemetry.addData("Total Score", String.format("%.2f", best.score));
        telemetry.update();
    }

    public void setPipeline(int pipeline) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
    }
}
