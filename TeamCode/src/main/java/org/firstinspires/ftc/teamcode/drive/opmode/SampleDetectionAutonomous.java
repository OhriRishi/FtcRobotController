package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.drive.opmode.DetectionParameters;
import java.util.List;

/**
 * Autonomous OpMode that uses sample color detection for navigation
 *
 * This OpMode demonstrates how to use the SampleColorDetector in an autonomous
 * program to detect and navigate to colored samples.
 *
 * The robot will:
 * 1. Initialize the vision system
 * 2. Scan for samples of the target color
 * 3. Navigate to the closest detected sample
 * 4. Perform a pickup action (placeholder)
 * 5. Repeat or end based on game strategy
 */
@Autonomous(name = "Sample Detection Auto", group = "Vision")
public class SampleDetectionAutonomous extends LinearOpMode {

    // Vision components
    private VisionPortal visionPortal;
    private ColorDetectionProcessor colorProcessor;
    private DetectionParameters detectionParams;

    // Drive motors (adjust names based on your robot configuration)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Timing and state
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime scanTimer = new ElapsedTime();

    // Autonomous parameters
    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.4;
    private static final double TARGET_DISTANCE_INCHES = 12.0;  // Stop this far from sample
    private static final double POSITION_TOLERANCE_INCHES = 2.0;
    private static final double ANGLE_TOLERANCE_DEGREES = 10.0;
    private static final double MAX_SCAN_TIME_SECONDS = 5.0;

    // Target color for this autonomous run
    private DetectionParameters.ColorMode targetColor = DetectionParameters.ColorMode.RED;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Initialize vision system
        initializeVision();

        // Display initialization status
        telemetry.addLine("Sample Detection Autonomous Initialized");
        telemetry.addData("Target Color", targetColor);
        telemetry.addLine("Ready to start!");
        telemetry.update();

        // Wait for start
        waitForStart();
        runtime.reset();

        // Main autonomous sequence
        while (opModeIsActive()) {
            // Step 1: Scan for samples
            SamplePose targetSample = scanForSamples();

            if (targetSample != null) {
                telemetry.addLine("Sample detected! Navigating...");
                telemetry.addData("Target", targetSample.toString());
                telemetry.update();

                // Step 2: Navigate to the sample
                boolean reachedTarget = navigateToSample(targetSample);

                if (reachedTarget) {
                    // Step 3: Perform pickup action
                    performPickupAction();

                    // Step 4: Continue or end based on strategy
                    telemetry.addLine("Sample collected! Mission complete.");
                    telemetry.update();
                    break;  // End autonomous for this example
                } else {
                    telemetry.addLine("Failed to reach target. Scanning again...");
                    telemetry.update();
                }
            } else {
                telemetry.addLine("No samples detected. Scanning...");
                telemetry.update();

                // Rotate to scan different areas
                rotateToScan();
            }

            // Safety timeout
            if (runtime.seconds() > 25.0) {
                telemetry.addLine("Autonomous timeout reached.");
                telemetry.update();
                break;
            }

            sleep(100);  // Small delay
        }

        // Stop all motors
        stopAllMotors();

        // Cleanup
        cleanup();
    }

    /**
     * Initialize robot hardware
     */
    private void initializeHardware() {
        // Initialize drive motors (adjust names based on your configuration)
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set motor directions (adjust based on your robot)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to brake when power is zero
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Initialize the vision system
     */
    private void initializeVision() {
        // Create detection parameters
        detectionParams = new org.firstinspires.ftc.teamcode.drive.opmode.DetectionParameters();
        detectionParams.currentColorMode = targetColor;

        // Create the color detection processor
        colorProcessor = new ColorDetectionProcessor(detectionParams, telemetry);

        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new android.util.Size(
                DetectionParameters.CAMERA_RESOLUTION_WIDTH,
                DetectionParameters.CAMERA_RESOLUTION_HEIGHT
        ));
        builder.addProcessor(colorProcessor);
        builder.enableLiveView(false);  // Disable live view for autonomous

        visionPortal = builder.build();
    }

    /**
     * Scan for samples and return the best target
     */
    private SamplePose scanForSamples() {
        scanTimer.reset();

        while (scanTimer.seconds() < MAX_SCAN_TIME_SECONDS && opModeIsActive()) {
            List<SamplePose> detectedSamples = colorProcessor.getDetectedSamples();

            if (!detectedSamples.isEmpty()) {
                // Find the closest valid sample
                SamplePose closestSample = null;
                double closestDistance = Double.MAX_VALUE;

                for (SamplePose sample : detectedSamples) {
                    if (sample.isValid()) {
                        double distance = sample.getDistanceFromCamera();
                        if (distance < closestDistance) {
                            closestDistance = distance;
                            closestSample = sample;
                        }
                    }
                }

                if (closestSample != null) {
                    return closestSample;
                }
            }

            sleep(50);  // Small delay between scans
        }

        return null;  // No samples found
    }

    /**
     * Navigate to a detected sample using corrected pinhole camera model
     */
    private boolean navigateToSample(SamplePose targetSample) {
        ElapsedTime navigationTimer = new ElapsedTime();
        navigationTimer.reset();

        while (navigationTimer.seconds() < 10.0 && opModeIsActive()) {
            // Get current detection (sample may have moved or detection may have updated)
            List<SamplePose> currentDetections = colorProcessor.getDetectedSamples();
            SamplePose currentTarget = findClosestSample(currentDetections, targetSample);

            if (currentTarget == null) {
                // Lost the target
                stopAllMotors();
                return false;
            }

            // Calculate navigation commands using the corrected pose data
            // The yInches represents forward distance, xInches represents lateral offset
            double forwardDistance = currentTarget.yInches;  // Forward distance to target
            double lateralOffset = currentTarget.xInches;    // Lateral offset (positive = right)
            double distanceToTarget = currentTarget.getDistanceFromCamera();  // Total distance
            double angleToTarget = currentTarget.getAngleToTarget();         // Angle to target

            // Check if we've reached the target
            if (Math.abs(forwardDistance - TARGET_DISTANCE_INCHES) < POSITION_TOLERANCE_INCHES &&
                    Math.abs(lateralOffset) < POSITION_TOLERANCE_INCHES) {
                stopAllMotors();
                return true;
            }

            // Calculate motor powers for navigation
            double forwardPower = 0.0;
            double turnPower = 0.0;

            // Forward/backward movement based on Y distance (forward distance)
            if (forwardDistance > TARGET_DISTANCE_INCHES + POSITION_TOLERANCE_INCHES) {
                forwardPower = DRIVE_SPEED;
            } else if (forwardDistance < TARGET_DISTANCE_INCHES - POSITION_TOLERANCE_INCHES) {
                forwardPower = -DRIVE_SPEED;
            }

            // Turning movement based on angle to target
            if (Math.abs(angleToTarget) > ANGLE_TOLERANCE_DEGREES) {
                turnPower = Math.signum(angleToTarget) * TURN_SPEED;
            }

            // Apply motor powers
            setDrivePowers(forwardPower, turnPower);

            // Update telemetry with corrected coordinate system
            telemetry.addData("Forward Distance", "%.1f inches", forwardDistance);
            telemetry.addData("Lateral Offset", "%.1f inches", lateralOffset);
            telemetry.addData("Total Distance", "%.1f inches", distanceToTarget);
            telemetry.addData("Angle to Target", "%.1f degrees", angleToTarget);
            telemetry.addData("Forward Power", "%.2f", forwardPower);
            telemetry.addData("Turn Power", "%.2f", turnPower);
            telemetry.update();

            sleep(50);
        }

        stopAllMotors();
        return false;  // Navigation timeout
    }

    /**
     * Find the sample closest to a reference sample
     */
    private SamplePose findClosestSample(List<SamplePose> samples, SamplePose reference) {
        SamplePose closest = null;
        double closestDistance = Double.MAX_VALUE;

        for (SamplePose sample : samples) {
            if (sample.isValid()) {
                double dx = sample.xInches - reference.xInches;
                double dy = sample.yInches - reference.yInches;
                double distance = Math.sqrt(dx * dx + dy * dy);

                if (distance < closestDistance) {
                    closestDistance = distance;
                    closest = sample;
                }
            }
        }

        return closest;
    }

    /**
     * Set drive motor powers for tank drive
     */
    private void setDrivePowers(double forward, double turn) {
        double leftPower = forward + turn;
        double rightPower = forward - turn;

        // Normalize powers if they exceed 1.0
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
    }

    /**
     * Stop all drive motors
     */
    private void stopAllMotors() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /**
     * Rotate to scan for samples
     */
    private void rotateToScan() {
        setDrivePowers(0, TURN_SPEED * 0.5);
        sleep(500);
        stopAllMotors();
    }

    /**
     * Perform sample pickup action (placeholder)
     */
    private void performPickupAction() {
        // Add your sample pickup code here
        // This might involve:
        // - Lowering an intake mechanism
        // - Running intake motors
        // - Raising the mechanism
        // - Storing the sample

        telemetry.addLine("Performing pickup action...");
        telemetry.update();
        sleep(2000);  // Placeholder delay
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
