package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static java.lang.StrictMath.PI;
import static java.lang.StrictMath.abs;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.IMG_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.IMG_WIDTH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.findNLargestContours;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.pixels2Degrees;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.getDistance2Tower;
import static org.firstinspires.ftc.utilities.Dash_AutoAim.MAX_H;
import static org.firstinspires.ftc.utilities.Dash_AutoAim.MAX_S;
import static org.firstinspires.ftc.utilities.Dash_AutoAim.MAX_V;
import static org.firstinspires.ftc.utilities.Dash_AutoAim.MIN_H;
import static org.firstinspires.ftc.utilities.Dash_AutoAim.MIN_S;
import static org.firstinspires.ftc.utilities.Dash_AutoAim.MIN_V;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.PS_LEFT_DIST;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.PS_CENTER_DIST;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.PS_RIGHT_DIST;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.SHOOTER_OFFSET_DISTANCE;
import static org.firstinspires.ftc.utilities.Dash_AutoAim.blur;
import static org.firstinspires.ftc.utilities.Dash_AutoAim.dilate_const;
import static org.firstinspires.ftc.utilities.Dash_AutoAim.erode_const;
import static org.firstinspires.ftc.utilities.Dash_AutoAim.goalWidth;
import static org.firstinspires.ftc.utilities.Dash_AutoAim.horizonLineRatio;
import static org.firstinspires.ftc.utilities.MathUtils.degATan;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.line;
import static org.opencv.imgproc.Imgproc.putText;
import static org.opencv.imgproc.Imgproc.rectangle;

public class AutoAimPipeline extends OpenCvPipeline {
    private boolean viewportPaused;


    private double degreeError = 0;

    // Init mats here so we don't repeat
    private Mat modified = new Mat();
    private Mat output = new Mat();
    private Mat hierarchy = new Mat();
    private List<MatOfPoint> new_contours;
    private List<MatOfPoint> contours;

    // Thresholding values
    Scalar MIN_HSV, MAX_HSV;

    // Rectangle settings
    private Scalar color = new Scalar(255, 0, 255);
    private int thickness = 2;
    private int font = FONT_HERSHEY_COMPLEX;
    public boolean isTowerFound;
    private double goalDistance;

    @Override
    public Mat processFrame(Mat input) {

        // Get height and width
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        // Take bottom portion
        double horizonY = (int) IMG_HEIGHT * horizonLineRatio;
        Rect upperRect = new Rect(new Point(0, 0), new Point(IMG_WIDTH, horizonY));
        input = input.submat(upperRect);

        // Copy to output
        input.copyTo(output);

        // Convert & Copy to outPut image
        cvtColor(input, modified, Imgproc.COLOR_RGB2HSV);

        // Blurring
        GaussianBlur(modified, modified, new Size(blur, blur), 0);

        // Thresholding
        MIN_HSV = new Scalar(MIN_H, MIN_S, MIN_V);
        MAX_HSV = new Scalar(MAX_H, MAX_S, MAX_V);
        inRange(modified, MIN_HSV, MAX_HSV, modified);

        // Erosion and Dilation
        erode(modified, modified, new Mat(erode_const, erode_const, CV_8U));
        dilate(modified, modified, new Mat(dilate_const, dilate_const, CV_8U));

        // Find contours of goal
        contours = new ArrayList<>();
        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0) { isTowerFound = false; return output; }
        else isTowerFound = true;

        // Retrieve goal contours
        new_contours = findNLargestContours(2, contours);

        // Get goalRectangle
        Rect goalRect = getGoalRect(new_contours);
        rectangle(output, goalRect, color, thickness);
        
        goalDistance = getDistance2Tower(goalRect.y);


        // Calculate error
        int center_x = goalRect.x + (goalRect.width / 2);
        int center_y = goalRect.y + (goalRect.height / 2);
        Point center = new Point(center_x, center_y);
        double pixel_error = (IMG_WIDTH / 2) - center_x;
        degreeError = pixels2Degrees(pixel_error);
        line(output, center, new Point(center_x + pixel_error, center_y), new Scalar(0, 0, 255), thickness);
        

        Point text_center = new Point(5, IMG_HEIGHT - 50);
        putText(output, "Degree Error: " + degreeError, text_center, font, 0.4, new Scalar(255, 255, 0));
        putText(output, "Pixel Error: " + pixel_error, new Point(5, IMG_HEIGHT - 40), font, 0.4, new Scalar(255, 255, 0));


        // Release all captures
        input.release();
        releaseAllCaptures();

        // Return altered image
        return output;

    }


    private Rect getGoalRect(List<MatOfPoint> contours) {

        // Return first contour if there is only one
        Rect goalRect = boundingRect(contours.get(0));

        // Extrapolate overarching rectangle if there are two
        if (contours.size() == 2) {

            // Init coords of both rectangles
            Rect left = new Rect(0, 0, 0, 0);
            Rect right = new Rect(0, 0, 0, 0);

            // Get bounding rects of second rectangle
            Rect secondRect = boundingRect(contours.get(1));

            // Check second rect is within goal width
            int diff = abs(goalRect.x - secondRect.x);
            if (diff > goalWidth) return goalRect;

            // Check which side rectangles are on, and calculate surrounding box
            if (goalRect.x < secondRect.x) {
                left.x = goalRect.x;
                left.y = goalRect.y;
                right.x = secondRect.x;
                right.y = secondRect.y;
                right.width = secondRect.width;
                right.height = secondRect.height;
            } else {
                left.x = secondRect.x;
                left.y = secondRect.y;
                right.x = goalRect.x;
                right.y = goalRect.y;
                right.width = goalRect.width;
                right.height = goalRect.height;
            }
            goalRect.x = left.x;
            goalRect.y = left.y;
            goalRect.width = abs(right.x - left.x) + right.width;
            goalRect.height = abs(right.y - left.y) + right.height;
        }

        return goalRect;
    }
    
    public double distance2Goal() {
        return goalDistance;
    }
    
    public double shooterOffsetAngle(){
        return degATan(SHOOTER_OFFSET_DISTANCE, goalDistance);
    }

    public double getDegreeError(){
        return degreeError * .8;
    }
    
    public double getPSDegreeError(PowerShot powerShot){
        double yDistance = goalDistance * Math.cos((Sensors.gyro.modAngle() + degreeError - 90) * (PI / 180));
        double xDistance = goalDistance * Math.sin((Sensors.gyro.modAngle() + degreeError - 90) * (PI / 180));
        double dDistance;
        switch (powerShot) {
            case PS_LEFT:
                dDistance = xDistance - PS_LEFT_DIST;
                break;
            case PS_CENTER:
                dDistance = xDistance - PS_CENTER_DIST;
                break;
            case PS_RIGHT:
                dDistance = xDistance - PS_RIGHT_DIST;
                break;
            default:
                dDistance = 0;
        }
        return (180 / PI) * Math.atan2(dDistance, yDistance) + 90;
    }

    public void releaseAllCaptures(){
        modified.release();
        hierarchy.release();
        if (contours != null){
            for (MatOfPoint cnt : contours){
                cnt.release();
            }
        }
    }
    
    public enum PowerShot{
        PS_LEFT,
        PS_CENTER,
        PS_RIGHT
    }

    @Override
    public void onViewportTapped() {
        viewportPaused = !viewportPaused;
        if (viewportPaused)     VisionUtils.webcam.pauseViewport();
        else                    VisionUtils.webcam.resumeViewport();
    }
}