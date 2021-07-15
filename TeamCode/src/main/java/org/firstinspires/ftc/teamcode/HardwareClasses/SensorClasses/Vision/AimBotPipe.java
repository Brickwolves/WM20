package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.utilities.RingBuffer;
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

import static java.lang.Math.round;
import static java.lang.StrictMath.PI;
import static java.lang.StrictMath.abs;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.DEBUG_MODE_ON;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.INIT_COMPLETED;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.INIT_RECT_SIDELENGTH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.MARGINS;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.MAX_H;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.MAX_S;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.MAX_V;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.MIN_H;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.MIN_S;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.MIN_V;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.blur;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.dilate_const;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.erode_const;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Dash_AimBot.goalWidth;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.AXES.X;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.IMG_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.IMG_WIDTH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.PS_CENTER_DIST;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.PS_CLOSE_DIST;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.PS_FAR_DIST;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.RECT_OPTION.AREA;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.TOWER_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.pixels2Degrees;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.VisionUtils.sortRectsByMaxOption;
import static org.firstinspires.ftc.utilities.Dash_AutoAim.conversion_factor;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.Core.mean;
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
//import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
//import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision.Deprecated.Dash_Shooter.SHOOTER_COEFF;

public class AimBotPipe extends OpenCvPipeline {
    private boolean viewportPaused;

    // Goal Variables
    private double goalDegreeError = 0;
    private boolean goalFound = false;
    private Rect goalRect = new Rect(0, 0, 0, 0);

    // When we turn, tower distances are a bit sporadic so we use a ringbuffer to average it out
    private double towerDistance = 0;
    private double distanceSum = 0;
    private RingBuffer<Double> distanceBuffer = new RingBuffer<>(4, 0.0);

    // Init mats here so we don't repeat
    private Mat modified = new Mat();
    private Mat output = new Mat();
    private Mat hierarchy = new Mat();
    private List<MatOfPoint> contours;

    // Rectangle settings
    private Scalar color = new Scalar(255, 0, 255);
    private int thickness = 2;
    private int font = FONT_HERSHEY_COMPLEX;


    private Scalar MAX_HSV = new Scalar(0, 0, 0);
    private Scalar MIN_HSV = new Scalar(0, 0, 0);

    private ElapsedTime time = new ElapsedTime();


    @Override
    public Mat processFrame(Mat input) {

        output = (!INIT_COMPLETED) ? initPipe(input) : regPipe(input);
        return output;
    }

    public void switch2AutoCalibrate(){
        INIT_COMPLETED = false;
    }
    public void switch2Regular(){
        INIT_COMPLETED = true;
    }

    public void updateHSV(Mat img, Rect crop){

        img.submat(crop);
        Scalar meanHSV = mean(img);
        for (int i=0; i < 3; i++){
            MAX_HSV.val[i] = meanHSV.val[i] + MARGINS[i];
            MIN_HSV.val[i] = meanHSV.val[i] - MARGINS[i];
        }
        //multTelemetry.addData("HSV", meanHSV.toString());

    }

    public Mat initPipe(Mat input){

        // Get height and width
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        // Copy to output
        input.copyTo(output);

        // Convert & Copy to outPut image
        cvtColor(input, modified, conversion_factor);

        // Blurring
        GaussianBlur(modified, modified, new Size(blur, blur), 0);

        // Retrieve initRect
        int sideLength = INIT_RECT_SIDELENGTH;
        int x = (int) (round(IMG_WIDTH / 2) - round(sideLength/2.0));
        int y = (int) (round(IMG_HEIGHT / 2) - round(sideLength/2.0));
        Rect initRect = new Rect(x, y, sideLength, sideLength);

        // Calc aveHSV w/i initRect
        updateHSV(modified, initRect);

        // Log the rect for driver-placement
        rectangle(output, initRect, color, thickness);

        // Log time left
        String timeStr = "" + time.seconds();
        double mx = IMG_WIDTH - 50;
        double my = IMG_HEIGHT - 50;
        putText(output, timeStr, new Point(mx, my), font, 0.4, color);

        return output;
    }

    public Mat regPipe(Mat input){

        // Get height and width
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        // Copy to output
        input.copyTo(output);

        // Convert & Copy to outPut image
        cvtColor(input, modified, conversion_factor);

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

        // Check if no goal is found
        if (contours.size() == 0) {
            goalFound = false;
            goalDegreeError = 0;
            towerDistance = 0;
            return output;
        }
        goalFound = true;

        // Retrieve all rects
        List<Rect> rects = new ArrayList<>();
        for (int i=0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }

        // Retrieve goal contours and make into one large rectangle
        List<Rect> largest_rects = sortRectsByMaxOption(2, AREA, rects);
        goalRect = mergeRects(largest_rects);

        // Calculate Center
        int center_x = goalRect.x + (goalRect.width / 2);
        int center_y = goalRect.y + (goalRect.height / 2);
        Point center = new Point(center_x, center_y);

        // Calculate Error
        double pixel_error = (IMG_WIDTH / 2) - center_x;
        goalDegreeError = pixels2Degrees(pixel_error, X);
        towerDistance = getDistance2Tower();

        // Logging Shapes and Degree & Pixel Data
        rectangle(output, goalRect, color, thickness);
        line(output, center, new Point(center_x + pixel_error, center_y), new Scalar(0, 0, 255), thickness);
        Point text_center = new Point(5, IMG_HEIGHT - 50);
        putText(output, "Degree Error: " + goalDegreeError, text_center, font, 0.4, new Scalar(255, 255, 0));
        putText(output, "Pixel Error: " + pixel_error, new Point(5, IMG_HEIGHT - 40), font, 0.4, new Scalar(255, 255, 0));


        /*
        // Release all captures
        input.release();
        releaseAllCaptures();
         */

        // Return altered image
        if (DEBUG_MODE_ON) return modified;
        return output;
    }


    public double getPSDegreeError(VisionUtils.PowerShot powerShot){
        double yDistance = towerDistance * Math.cos((Sensors.gyro.modAngle() + goalDegreeError - 90) * (PI / 180));
        double xDistance = towerDistance * Math.sin((Sensors.gyro.modAngle() + goalDegreeError - 90) * (PI / 180));
        double dDistance;
        switch (powerShot) {
            case PS_CLOSE:
                dDistance = xDistance - PS_CLOSE_DIST;
                break;
            case PS_MIDDLE:
                dDistance = xDistance - PS_CENTER_DIST;
                break;
            case PS_FAR:
                dDistance = xDistance - PS_FAR_DIST;
                break;
            default:
                dDistance = 0;
        }
        return (180 / PI) * Math.atan2(dDistance, yDistance) + 90;
    }

    public boolean isGoalFound(){
        return goalFound;
    }

    public double getRawTowerDegreeError(){
        return (isGoalFound()) ? goalDegreeError : 0;
    }

    public double getTowerDegreeError(){
        return (isGoalFound()) ? goalDegreeError * 0.8 : 0;
    }

    public double getDistance2Tower() {
        if (!isGoalFound() || goalRect.y == 0) return 0;
        double towerHeight = TOWER_HEIGHT - goalRect.y;
        double theta = (towerHeight / IMG_HEIGHT) * .75;
        double distance = 100/Math.tan(theta) - 0;
        distanceSum = distanceSum + distance - distanceBuffer.getValue(distance);
        return distanceSum / 4;
    }

    public Rect getGoalRect(){
        return  goalRect;
    }

    private Rect mergeRects(List<Rect> rects) {

        // Return first contour if there is only one
        Rect goalRect = rects.get(0);

        // Extrapolate overarching rectangle if there are two
        if (rects.size() == 2) {

            // Init coords of both rectangles
            Rect left = new Rect(0, 0, 0, 0);
            Rect right = new Rect(0, 0, 0, 0);

            // Get bounding rects of second rectangle
            Rect secondRect = rects.get(1);

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

    public void releaseAllCaptures(){
        modified.release();
        hierarchy.release();
        if (contours != null){
            for (MatOfPoint cnt : contours){
                cnt.release();
            }
        }
    }

    @Override
    public void onViewportTapped() {
        viewportPaused =        !viewportPaused;
        if (viewportPaused)     VisionUtils.webcam_front.pauseViewport();
        else                    VisionUtils.webcam_front.resumeViewport();
    }
}