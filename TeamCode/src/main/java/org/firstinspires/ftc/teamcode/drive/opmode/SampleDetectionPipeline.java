
package org.firstinspires.ftc.teamcode.drive.opmode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import com.acmerobotics.dashboard.config.Config;

@Config
public class SampleDetectionPipeline extends OpenCvPipeline {

    public enum ColorMode { RED, BLUE, YELLOW }
    private ColorMode colorMode = ColorMode.RED;
    public RotatedRect[] latestRects;
    public double[][] latestDistances;
    public double REAL_SAMPLE_LENGTH = 3.5;
    public double REAL_SAMPLE_WIDTH = 1.5;
    public double REAL_SAMPLE_HEIGHT = 1.5;
    public static double FOCAL_LEGNTH_X = 400;
    public static double FOCAL_LEGNTH_Y = 600;
    public static double HEIGHT_OF_CAMERA = 10; //Inches
    public static double CENTER_OF_IMAGE = 12; //Inches
    public static int lowH = 0;
    public static int lowS = 120;
    public static int lowV = 70;
    public static int highH = 10;
    public static int highS = 255;
    public static int highV = 255;

    public static int lowH2 = 170;
    public static int highH2 = 180;
    public static final double CAMERA_ANGLE = 40;

    private static final double CAMERA_FOV_HORIZONTAL = 78.0;
    private static final double CAMERA_FOV_VERTICAL = 43.0;
    private static final int CAMERA_RESOLUTION_WIDTH = 640;
    private static final int CAMERA_RESOLUTION_HEIGHT = 480;
    private static final double SAMPLE_WIDTH_INCHES = 1.5;

    public void setColorMode(ColorMode mode) {
        this.colorMode = mode;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat mask = new Mat();
        switch (colorMode) {
            case RED:
                Mat lowerRed = new Mat();
                Mat upperRed = new Mat();
                Core.inRange(hsv, new Scalar(lowH, lowS, lowV), new Scalar(highH, highS, highV), lowerRed);
                Core.inRange(hsv, new Scalar(lowH2, lowS, lowV), new Scalar(highH2, highS, highV), upperRed);
                Core.bitwise_or(lowerRed, upperRed, mask);
                break;
            case BLUE:
            case YELLOW:
                Core.inRange(hsv, new Scalar(lowH, lowS, lowV), new Scalar(highH, highS, highV), mask);
                break;
        }

        Imgproc.erode(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        Imgproc.dilate(mask, mask, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        ArrayList<RotatedRect> detectedRects = new ArrayList<>();
        ArrayList<double[]> distances = new ArrayList<>();
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) > 500) {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                detectedRects.add(rect);
                Point[] vertices = new Point[4];
                rect.points(vertices);
                for (int j = 0; j < 4; j++) {
                    Imgproc.line(input, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
                }
                double distance3D = (REAL_SAMPLE_LENGTH * FOCAL_LEGNTH_X) / Math.max(rect.size.width, rect.size.height);
                double img_center_x = CAMERA_RESOLUTION_WIDTH / 2;
                double verticalFovRad = Math.toRadians(CAMERA_FOV_VERTICAL);
                double pixelsPerRad = CAMERA_RESOLUTION_HEIGHT / verticalFovRad;
                double img_center_y = CAMERA_RESOLUTION_HEIGHT / 2.0;
                double pixel_from_center_x = rect.center.x - img_center_x;
                double pixel_from_center_y = rect.center.y - img_center_y; //Invert because y is flipped
                double angle_horizontal = Math.atan2(pixel_from_center_x, FOCAL_LEGNTH_X);
                double x_inches_x = distance3D * Math.sin(angle_horizontal);
                double y_inches_x = distance3D * Math.cos(angle_horizontal);

                double verticalAngle = Math.atan2(pixel_from_center_y, FOCAL_LEGNTH_Y);
                double y_inches_y = HEIGHT_OF_CAMERA * Math.cos(verticalAngle);



                // Angle from camera's optical axis to object (add camera tilt)
                double angle_to_object = Math.toRadians(CAMERA_ANGLE) + (pixel_from_center_y / pixelsPerRad);

                // True ground distance from robot to object
                double ground_distance = HEIGHT_OF_CAMERA / Math.tan(angle_to_object);


                double angle = rect.angle;
                if(rect.size.width < rect.size.height){
                    angle += 90;
                }
                y_inches_y = ground_distance;
                double turretAngle = Math.toDegrees(Math.atan(x_inches_x/y_inches_y));
                distances.add(new double[]{x_inches_x, angle,  y_inches_y, turretAngle});
            }
        }

        latestRects = detectedRects.toArray(new RotatedRect[0]);
        latestDistances = distances.toArray(new double[0][]);
        return input;
    }
}
