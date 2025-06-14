package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;

// reference diagram: https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
@Config
public class LimelightConstants {
    public static double IDEAL_ASPECT_RATIO = 3.5 / 1.5; // Expected width:height ratio for a properly aligned sample
    public static double IDEAL_Y = 10;
    public static int PIPELINE = 0;
    public static double LIME_LIGHT_MOUNT_ANGLE = 35; // a1
    public static double LIME_LIGHT_LENS_HEIGHT_INCHES = 7.00525945; // h1
    public static double LIME_LIGHT_OFFSET = 6.2; // h1
    public static double SAMPLE_HEIGHT_INCHES = 0; // h2
    public static double TELESCOPE_OFFSET = 2; // h2
    public static double X_WEIGHT = 2;
    public static double Y_WEIGHT = 3;
    public static double ROT_WEIGHT = 20;
}