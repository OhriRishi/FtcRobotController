package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.ColorDetectionProcessor;
import org.firstinspires.ftc.teamcode.drive.opmode.DetectionParameters;
import org.firstinspires.ftc.teamcode.drive.opmode.SamplePose;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

/**
 * FTC OpMode for Sample Color Detection
 *
 * This OpMode implements computer vision-based detection of colored samples
 * using OpenCV and the FTC VisionPortal API.
 *
 * Controls:
 * - Gamepad1 X: Switch to RED detection
 * - Gamepad1 Y: Switch to YELLOW detection
 * - Gamepad1 B: Switch to BLUE detection
 * - Gamepad1 A: Toggle edge detection
 * - Gamepad1 Start: Save parameters (future feature)
 * - Gamepad1 Back: Reset parameters (future feature)
 *
 * The OpMode displays detection results via telemetry and can be used
 * for autonomous navigation or manual verification of sample positions.
 */
@TeleOp(name = "Sample Color Detector", group = "Vision")
public class SampleColorDetectorReal extends LinearOpMode {

    // Vision components
    private VisionPortal visionPortal;
    private ColorDetectionProcessor colorProcessor;
    private DetectionParameters detectionParams;

    // Timing
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime buttonCooldown = new ElapsedTime();
    private static final double BUTTON_COOLDOWN_MS = 300; // Prevent button spam

    // State tracking
    private DetectionParameters.ColorMode lastColorMode;
    private boolean lastEdgeDetectionState;

    @Override
    public void runOpMode() {
        // Initialize the detection system
        initializeVision();

        // Display initialization status
        telemetry.addLine("Sample Color Detector Initialized");
        telemetry.addLine("Ready to start!");
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("X - Red Detection");
        telemetry.addLine("Y - Yellow Detection");
        telemetry.addLine("B - Blue Detection");
        telemetry.addLine("A - Toggle Edge Detection");
        telemetry.update();

        // Wait for start
        waitForStart();
        runtime.reset();

        // Main execution loop
        while (opModeIsActive()) {
            // Handle user input
            handleGamepadInput();

            // Get detection results
            List<SamplePose> detectedSamples = colorProcessor.getDetectedSamples();

            // Update telemetry with results
            updateTelemetry(detectedSamples);

            // Small delay to prevent excessive CPU usage
            sleep(50);
        }

        // Cleanup
        cleanup();
    }

    /**
     * Initialize the vision system
     */
    private void initializeVision() {
        // Create detection parameters
        detectionParams = new DetectionParameters();

        // Create the color detection processor
        colorProcessor = new ColorDetectionProcessor(detectionParams, telemetry);

        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (assumes webcam named "Webcam 1")
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set camera resolution
        builder.setCameraResolution(new android.util.Size(
                DetectionParameters.CAMERA_RESOLUTION_WIDTH,
                DetectionParameters.CAMERA_RESOLUTION_HEIGHT
        ));

        // Add the processor
        builder.addProcessor(colorProcessor);

        // Enable camera stream (for debugging)
        builder.enableLiveView(true);

        // Build the vision portal
        visionPortal = builder.build();

        // Initialize state tracking
        lastColorMode = detectionParams.currentColorMode;
        lastEdgeDetectionState = detectionParams.useEdgeDetection;
    }

    /**
     * Handle gamepad input for controlling detection parameters
     */
    private void handleGamepadInput() {
        // Only process input if cooldown has expired
        if (buttonCooldown.milliseconds() < BUTTON_COOLDOWN_MS) {
            return;
        }

        boolean inputProcessed = false;

        // Color mode selection
        if (gamepad1.x) {
            detectionParams.currentColorMode = DetectionParameters.ColorMode.RED;
            inputProcessed = true;
        } else if (gamepad1.y) {
            detectionParams.currentColorMode = DetectionParameters.ColorMode.YELLOW;
            inputProcessed = true;
        } else if (gamepad1.b) {
            detectionParams.currentColorMode = DetectionParameters.ColorMode.BLUE;
            inputProcessed = true;
        }

        // Edge detection toggle
        if (gamepad1.a) {
            detectionParams.useEdgeDetection = !detectionParams.useEdgeDetection;
            inputProcessed = true;
        }

        // Parameter adjustments using dpad and triggers
        if (gamepad1.dpad_up) {
            detectionParams.minContourArea += 100;
            inputProcessed = true;
        } else if (gamepad1.dpad_down) {
            detectionParams.minContourArea = Math.max(100, detectionParams.minContourArea - 100);
            inputProcessed = true;
        }

        if (gamepad1.dpad_right) {
            detectionParams.cannyThreshold1 += 5;
            detectionParams.cannyThreshold2 += 5;
            inputProcessed = true;
        } else if (gamepad1.dpad_left) {
            detectionParams.cannyThreshold1 = Math.max(5, detectionParams.cannyThreshold1 - 5);
            detectionParams.cannyThreshold2 = Math.max(10, detectionParams.cannyThreshold2 - 5);
            inputProcessed = true;
        }

        // Erosion/dilation adjustments
        if (gamepad1.right_trigger > 0.5) {
            detectionParams.erodeIterations = Math.min(10, detectionParams.erodeIterations + 1);
            inputProcessed = true;
        } else if (gamepad1.left_trigger > 0.5) {
            detectionParams.erodeIterations = Math.max(0, detectionParams.erodeIterations - 1);
            inputProcessed = true;
        }

        // Reset cooldown if input was processed
        if (inputProcessed) {
            buttonCooldown.reset();
        }
    }

    /**
     * Update telemetry with detection results and system status
     */
    private void updateTelemetry(List<SamplePose> detectedSamples) {
        // Clear previous telemetry
        telemetry.clear();

        // System status
        telemetry.addLine("=== Sample Color Detector ===");
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("Color Mode", detectionParams.currentColorMode);
        telemetry.addData("Edge Detection", detectionParams.useEdgeDetection ? "ON" : "OFF");
        telemetry.addLine();

        // Detection parameters
        telemetry.addLine("=== Parameters ===");
        telemetry.addData("Min Area", "%.0f px²", detectionParams.minContourArea);
        telemetry.addData("Erode Iterations", detectionParams.erodeIterations);
        telemetry.addData("Canny Thresholds", "%.0f, %.0f",
                detectionParams.cannyThreshold1, detectionParams.cannyThreshold2);
        telemetry.addLine();

        // Detection results
        telemetry.addLine("=== Detections ===");
        telemetry.addData("Samples Found", detectedSamples.size());

        for (int i = 0; i < detectedSamples.size(); i++) {
            SamplePose pose = detectedSamples.get(i);
            telemetry.addLine(String.format("Sample %d:", i + 1));
            telemetry.addData("  Position", "X=%.1f\", Y=%.1f\"", pose.xInches, pose.yInches);
            telemetry.addData("  Angle", "%.1f°", pose.angleDegrees);
            telemetry.addData("  Confidence", "%.2f", pose.confidence);
            telemetry.addData("  Valid", pose.isValid() ? "YES" : "NO");
        }

        if (detectedSamples.isEmpty()) {
            telemetry.addLine("No samples detected");
        }

        telemetry.addLine();
        telemetry.addLine("=== Controls ===");
        telemetry.addLine("X/Y/B - Color Mode");
        telemetry.addLine("A - Toggle Edge Detection");
        telemetry.addLine("DPad - Adjust Parameters");
        telemetry.addLine("Triggers - Erosion");

        // Update the display
        telemetry.update();
    }

    /**
     * Get the best detected sample (highest confidence)
     */
    public SamplePose getBestDetection() {
        List<SamplePose> samples = colorProcessor.getDetectedSamples();

        SamplePose best = null;
        double bestConfidence = 0.0;

        for (SamplePose sample : samples) {
            if (sample.isValid() && sample.confidence > bestConfidence) {
                best = sample;
                bestConfidence = sample.confidence;
            }
        }

        return best;
    }

    /**
     * Get all valid detections
     */
    public List<SamplePose> getValidDetections() {
        List<SamplePose> samples = colorProcessor.getDetectedSamples();
        List<SamplePose> validSamples = new java.util.ArrayList<>();

        for (SamplePose sample : samples) {
            if (sample.isValid()) {
                validSamples.add(sample);
            }
        }

        return validSamples;
    }

    /**
     * Cleanup resources
     */
    private void cleanup() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
