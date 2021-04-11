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

import static org.firstinspires.ftc.utilities.Dash_PSFinder.blur;
import static org.firstinspires.ftc.utilities.Dash_PSFinder.dilate_const;
import static org.firstinspires.ftc.utilities.Dash_PSFinder.erode_const;
import static org.firstinspires.ftc.utilities.Dash_PSFinder.MAX_Cb;
import static org.firstinspires.ftc.utilities.Dash_PSFinder.MAX_Cr;
import static org.firstinspires.ftc.utilities.Dash_PSFinder.MAX_Y;
import static org.firstinspires.ftc.utilities.Dash_PSFinder.MIN_Cb;
import static org.firstinspires.ftc.utilities.Dash_PSFinder.MIN_Cr;
import static org.firstinspires.ftc.utilities.Dash_PSFinder.MIN_Y;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.IMG_HEIGHT;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.IMG_WIDTH;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.findNLargestContours;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.findNLeftMostContours;
import static org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.VisionUtils.pixels2Degrees;
import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.putText;
import static org.opencv.imgproc.Imgproc.rectangle;

public class PSAimPipeline extends OpenCvPipeline {
    private boolean viewportPaused;


    private double degree_error = 0;
    private double[] ps_errors = {0, 0, 0};
    private int n_ps_found = 0;
    public boolean arePSFound;

    // Init mats here so we don't repeat
    private Mat modified = new Mat();
    private Mat output = new Mat();
    private Mat hierarchy = new Mat();
    private List<MatOfPoint> largest_contours;
    private List<MatOfPoint> contours;

    // Thresholding values
    Scalar MIN_YCrCb, MAX_YCrCb;

    // Rectangle settings
    private Scalar color = new Scalar(255, 0, 255);
    private int thickness = 2;
    private int font = FONT_HERSHEY_COMPLEX;

    public enum PS {
        LEFT, CENTER, RIGHT
    }

    public double getPSError(PS ps){
        if (n_ps_found < 3) return ps_errors[0];
        else if (n_ps_found == 3){
            switch (ps){
                case LEFT:
                    return ps_errors[0];
                case CENTER:
                    return ps_errors[1];
                case RIGHT:
                    return ps_errors[2];
            }
        }
        return 0;
    }

    @Override
    public Mat processFrame(Mat input) {

        // Get height and width
        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();

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


        // Find contours of goal
        contours = new ArrayList<>();
        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    
        ArrayList<Integer> indexes2Remove = new ArrayList<>();
        for (int i=0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            if (rect.width > rect.height) indexes2Remove.add(i);
        }
        for (int i =0; i < indexes2Remove.size(); i++){
            int index1 = indexes2Remove.get(i);
            if (contours.size() > 0) contours.remove(index1);
            else return output;
        
            // adjust other indexes that move down
            for (int j=0; j < indexes2Remove.size(); j++){
                int index2 = indexes2Remove.get(j);
                if (index1 < index2) indexes2Remove.set(j, index2 - 1);
            }
        }
        
        if (contours.size() == 0) return output;

        // Remove contour if it's the size of the FREAKING SCREEN
        int removeI = 0;
        boolean screenContourFound = false;
        for (int i=0; i < contours.size(); i++){
            if (contourArea(contours.get(i)) > 1000){
                removeI = i;
                screenContourFound = true;
            }
        }
        if (screenContourFound) contours.remove(removeI);

        if (contours.size() == 0){
            arePSFound = false;
            return output;
        }else{
            arePSFound = true;
        }


        // Retrieve powershot contours
        largest_contours = findNLargestContours(3, contours);

        n_ps_found = largest_contours.size();

        // Sort the contours from left to right
        largest_contours = findNLeftMostContours(3, largest_contours);

        int c = 0;
        for (MatOfPoint cnt : largest_contours){

            // Get and Draw PowerShot Rectangle
            Rect rect = boundingRect(cnt);
            rectangle(output, rect, color, thickness);

            // Find center
            int center_x = rect.x + (rect.width / 2);
            int center_y = rect.y + (rect.height / 2);
            Point center = new Point(center_x, center_y);

            // Calculate error
            double pixel_error = (IMG_WIDTH / 2) - center_x;
            degree_error = pixels2Degrees(pixel_error);
            ps_errors[c] = degree_error;

            // Visually identify power shots
            String msg = "" + c + "";
            //line(output, center, new Point(center_x + pixel_error, center_y), new Scalar(0, 0, 255), thickness);
            putText(output, msg, center, font, 0.5, color);
            c++;
        }

        return output;


        /*
        Point text_center = new Point(5, IMG_HEIGHT - 50);
        putText(output, "Degree Error: " + degree_error, text_center, font, 0.4, new Scalar(255, 255, 0));
        putText(output, "Pixel Error: " + pixel_error, new Point(5, IMG_HEIGHT - 40), font, 0.4, new Scalar(255, 255, 0));
         */


        /*
        // Release all captures
        input.release();
        releaseAllCaptures();

        // Return altered image
        return output;

         */


    }

    private Rect[] sortRectsByX(Rect[] rects){
        ArrayList<Integer> xcoords = new ArrayList<>();
        for (Rect rect : rects){
            xcoords.add(rect.x);
        }
        int leftI = getMinIndex(xcoords);
        xcoords.remove(leftI);
        int middleI = getMinIndex(xcoords);
        xcoords.remove(middleI);
        int rightI = xcoords.get(0);
        return new Rect[] {rects[leftI], rects[middleI], rects[rightI]};
    }

    private int getMinIndex(ArrayList<Integer> x){
        double beta = Integer.MAX_VALUE;
        int betaI= 0;
        for (int i=0; i < x.size(); i++){
            if (x.get(i) < beta){
                beta = x.get(i);
                betaI = i;
            }
        }
        return betaI;
    }

    public double getDegreeError(){
        return degree_error;
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
        viewportPaused = !viewportPaused;
        if (viewportPaused)  VisionUtils.webcam.pauseViewport();
        else                VisionUtils.webcam.resumeViewport();
    }
}