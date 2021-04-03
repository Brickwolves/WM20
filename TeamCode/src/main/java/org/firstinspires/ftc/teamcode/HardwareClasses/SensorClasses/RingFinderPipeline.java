package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

import org.firstinspires.ftc.teamcode.Autonomous.lupineAutos.VisionUtils;
import org.firstinspires.ftc.utilities.Dash_GoalFinder;
import org.opencv.core.Core;
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

import static org.firstinspires.ftc.teamcode.Autonomous.lupineAutos.VisionUtils.IMG_WIDTH;
import static org.firstinspires.ftc.teamcode.Autonomous.lupineAutos.VisionUtils.IMG_HEIGHT;
import static org.firstinspires.ftc.teamcode.Autonomous.lupineAutos.VisionUtils.RING_HEIGHT;
import static org.firstinspires.ftc.teamcode.Autonomous.lupineAutos.VisionUtils.findNWidestContours;
import static org.firstinspires.ftc.teamcode.Autonomous.lupineAutos.VisionUtils.getDistance2Object;
import static org.firstinspires.ftc.teamcode.Autonomous.lupineAutos.VisionUtils.pixels2Degrees;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.MAX_Cb;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.MAX_Cr;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.MAX_Y;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.MIN_Cb;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.MIN_Cr;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.MIN_Y;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.blur;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.dilate_const;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.erode_const;
import static org.firstinspires.ftc.utilities.Dash_RingFinder.horizonLineRatio;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.Core.rotate;
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

//Front Camera
//Back Camera

public class RingFinderPipeline extends OpenCvPipeline
{
    private boolean viewportPaused;

    // Constants
    private int ring_count = 0;
    private double degrees_error = 0;

    // Init mats here so we don't repeat
    private Mat modified = new Mat();
    private Mat output = new Mat();

    // Thresholding values
    Scalar MIN_YCrCb, MAX_YCrCb;

    // Rectangle settings
    private Scalar color = new Scalar(255, 0, 255);
    private int thickness = 2;
    private int font = FONT_HERSHEY_COMPLEX;

    @Override
    public Mat processFrame(Mat input)
    {

        // Rotate due to camera
        rotate(input, input, Core.ROTATE_90_CLOCKWISE);

        // Take bottom portion
        double horizonY = VisionUtils.IMG_HEIGHT * Dash_GoalFinder.horizonLineRatio;
        Rect bottomRect = new Rect(new Point(0, horizonY), new Point(IMG_WIDTH, IMG_HEIGHT));
        input = input.submat(bottomRect);

        // Copy to output
        input.copyTo(output);

        // Get height and width
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

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
        Mat hierarchy = new Mat();
        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // Check if we have detected any orange objects and assume ring_count is 0
        ring_count = 0;
        if (contours.size() > 0) {

            // Retrieve widest (closest) rect
            List<MatOfPoint> widest_contours = findNWidestContours(3, contours);
            MatOfPoint widest_contour = widest_contours.get(0);
            Rect widest_rect = boundingRect(widest_contour);

            // Check if it is below horizon line
            double horizonLine = VisionUtils.IMG_HEIGHT * horizonLineRatio;
            line(output, new Point(0, horizonLine), new Point(IMG_WIDTH, horizonLine), color, thickness);
            if (widest_rect.y < horizonLine) return output;

            // Calculate error
            int center_x = widest_rect.x + (widest_rect.width / 2);
            int center_y = widest_rect.y + (widest_rect.height / 2);
            Point center = new Point(center_x, center_y);
            double pixel_error = (VisionUtils.IMG_WIDTH / 2) - center_x;
            degrees_error = pixels2Degrees(pixel_error);
            line(output, center, new Point(center_x + pixel_error, center_y), new Scalar(0, 0, 255), thickness);

            // Log center
            //String coords = "(" + center_x + ", " + center_y + ")";
            //putText(output, coords, center, font, 0.5, color);

            Point text_center = new Point(5, IMG_HEIGHT - 50);
            putText(output, "Degree Error: " + degrees_error, text_center, font, 0.4, new Scalar(255, 255, 0));
            putText(output, "Pixel Error: " + pixel_error, new Point(5, IMG_HEIGHT - 40), font, 0.4, new Scalar(255, 255, 0));


            for (MatOfPoint cnt : widest_contours){
                Rect rect = boundingRect(cnt);
                rectangle(output, rect, color, thickness);
            }

            // Update ring count
            ring_count = (widest_rect.height < (0.5 * widest_rect.width)) ? 1 : 4;
            double distance2Ring = getDistance2Object(widest_rect.height, RING_HEIGHT);
        }

        // Return altered image
        return output;
    }

    public int getRingCount(){
        return ring_count;
    }


    @Override
    public void onViewportTapped() {
        viewportPaused = !viewportPaused;
        if (viewportPaused)     VisionUtils.webcam.pauseViewport();
        else                    VisionUtils.webcam.resumeViewport();
    }
}