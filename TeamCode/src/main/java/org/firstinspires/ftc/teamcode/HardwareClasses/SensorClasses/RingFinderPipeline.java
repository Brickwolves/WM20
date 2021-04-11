package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

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

import static org.firstinspires.ftc.utilities.Dash_RingFinder.*;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.MAX_Cb;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.MAX_Cr;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.MAX_Y;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.MIN_Cb;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.MIN_Cr;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.MIN_Y;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.blur;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.dilate_const;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.erode_const;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.IMG_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.IMG_WIDTH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.RING_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.findNWidestContours;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.getDistance2Tower;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.pixels2Degrees;
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

public class RingFinderPipeline extends OpenCvPipeline
{

    // Constants
    private int ring_count = 0;
    private double degrees_error = 0;

    // Init mats here so we don't repeat
    private Mat modified = new Mat();
    private Mat output = new Mat();
    private Mat hierarchy = new Mat();
    private List<MatOfPoint> contours;
    private MatOfPoint widest_rect;

    // Thresholding values
    Scalar MIN_YCrCb, MAX_YCrCb;

    // Rectangle settings
    private Scalar color = new Scalar(255, 0, 255);
    private int thickness = 2;
    private int font = FONT_HERSHEY_COMPLEX;

    @Override
    public Mat processFrame(Mat input)
    {

        // Get height and width
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

        // Take bottom portion
        double horizonY = (int) IMG_HEIGHT * horizonLineRatio;
        Rect bottomRect = new Rect(new Point(0, horizonY), new Point(IMG_WIDTH, IMG_HEIGHT));
        input = input.submat(bottomRect);

        // Copy to output
        input.copyTo(output);

        // Convert & Copy to outPut image
        cvtColor(input, modified, Imgproc.COLOR_RGB2YCrCb);

        // Blurring
        GaussianBlur(modified, modified, new Size(blur, blur), 0);

        // Thresholding
        MIN_YCrCb = new Scalar(MIN_Y, MIN_Cr, MIN_Cb);
        MAX_YCrCb = new Scalar(MAX_Y, MAX_Cr, MAX_Cb);
        inRange(modified, MIN_YCrCb, MAX_YCrCb, modified);

        // Erosion and Dilation
        erode(modified, modified, new Mat(erode_const, erode_const, CV_8U));
        dilate(modified, modified, new Mat(dilate_const, dilate_const, CV_8U));

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // Check if we have detected any orange objects and assume ring_count is 0
        ring_count = 0;
        if (contours.size() > 0) {

            // Retrieve widest (closest) rect
            List<MatOfPoint> widest_contours = findNWidestContours(3, contours);
            MatOfPoint widest_contour = widest_contours.get(0);
            Rect widest_rect = boundingRect(widest_contour);

            // Calculate error
            int center_x = widest_rect.x + (widest_rect.width / 2);
            int center_y = widest_rect.y + (widest_rect.height / 2);
            Point center = new Point(center_x, center_y);
            double pixel_error = (IMG_WIDTH / 2) - center_x;
            degrees_error = pixels2Degrees(pixel_error);

            // Update ring count
            ring_count = (widest_rect.height < (0.5 * widest_rect.width)) ? 1 : 4;

            // Get distance to ring
            

            // Box 3 closest rings
            for (MatOfPoint cnt : widest_contours){
                Rect rect = boundingRect(cnt);
                rectangle(output, rect, color, thickness);
            }


            /*

                            L O G G I N G

                                                       */

            // Log center
            //String coords = "(" + center_x + ", " + center_y + ")";
            //putText(output, coords, center, font, 0.5, color);

            // Log data on screen
            Point text_center = new Point(5, IMG_HEIGHT - 50);
            putText(output, "Degree Error: " + degrees_error, text_center, font, 0.4, new Scalar(255, 255, 0));
            putText(output, "Pixel Error: " + pixel_error, new Point(5, IMG_HEIGHT - 40), font, 0.4, new Scalar(255, 255, 0));
            line(output, center, new Point(center_x + pixel_error, center_y), new Scalar(0, 0, 255), thickness);

            /*
            Utils.multTelemetry.addData("Ring Count", ring_count);
            Utils.multTelemetry.addData("Pixel Error", pixel_error);
            Utils.multTelemetry.addData("Degree Error", degrees_error);
            Utils.multTelemetry.addData("IMU Angle", RingFinder.imu.getAngle());
            //Utils.multTelemetry.addData("Distance2Object", distance2Ring);
            Utils.multTelemetry.update();
             */
        }

        // Release all captures
        input.release();
        releaseAllCaptures();

        // Return altered image
        return output;
    }

    public void releaseAllCaptures(){
        modified.release();
        hierarchy.release();
        if (contours != null){
            for (MatOfPoint cnt : contours){
                cnt.release();
            }
        }

        if (widest_rect != null) widest_rect.release();
    }

    public int getRingCount(){
        return ring_count;
    }

    public double getRingAngle(){
        return degrees_error;
    }

    /*@Override
    public void onViewportTapped() {
        viewportPaused = !viewportPaused;
        if (viewportPaused)     webcam.pauseViewport();
        else                    webcam.resumeViewport();
    }*/
}