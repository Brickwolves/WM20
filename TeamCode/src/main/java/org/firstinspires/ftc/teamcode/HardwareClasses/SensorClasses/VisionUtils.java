package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.List;

import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.contourArea;

public class VisionUtils {

    public static OpenCvCamera webcam;
    public static double IMG_WIDTH = 320;
    public static double IMG_HEIGHT = 240;
    public static final double FOV = 72;
    public static final double RING_HEIGHT = 20;
    public static final double TOWER_HEIGHT = 220;
    
    public static double PS_LEFT_DIST = 41;
    public static double PS_CENTER_DIST = 64;
    public static double PS_RIGHT_DIST = 80;
    
    public static double SHOOTER_OFFSET_DISTANCE = 12;
    
    
    public static double getDistance2Tower(double yValue) {
        if (yValue == 0) return 0;
        double towerHeight = TOWER_HEIGHT - yValue;
        double theta = (towerHeight / IMG_HEIGHT) * .75;
        return 100/Math.tan(theta);
    }
    
    public static double pixels2Degrees(double pixels) {
        return pixels * (FOV / IMG_WIDTH);
    }

    public static enum CONTOUR_OPTION {
        AREA, WIDTH, HEIGHT
    }
    
    public static int findLeftMostContourIndex(List<MatOfPoint> contours){
        int index = 0;
        double minX = Integer.MAX_VALUE;
        for (int i=0; i < contours.size(); i++){
            MatOfPoint cnt = contours.get(i);
            double x = boundingRect(cnt).x;
            if (x < minX) {
                minX = x;
                index = i;
            }
        }
        return index;
    }
    
    public static List<MatOfPoint> findNLeftMostContours(int n, List<MatOfPoint> contours){
        List<MatOfPoint> widest_contours = new ArrayList<>();
        for (int j=0; j < n; j++){
            int largest_index = findLeftMostContourIndex(contours);
            widest_contours.add(contours.get(largest_index));
            
            contours.remove(largest_index);
            if (contours.size() == 0) break;
        }
        
        for (MatOfPoint cnt : contours){
            cnt.release();
        }
        
        return widest_contours;
    }

    public static int findWidestContourIndex(List<MatOfPoint> contours){
        int index = 0;
        double maxWidth = 0;
        for (int i=0; i < contours.size(); i++){
            MatOfPoint cnt = contours.get(i);
            double width = boundingRect(cnt).width;
            if (width > maxWidth) {
                maxWidth = width;
                index = i;
            }
        }
        return index;
    }

    public static List<MatOfPoint> findNWidestContours(int n, List<MatOfPoint> contours){
        List<MatOfPoint> widest_contours = new ArrayList<>();
        for (int j=0; j < n; j++){
            int largest_index = findWidestContourIndex(contours);
            widest_contours.add(contours.get(largest_index));

            contours.remove(largest_index);
            if (contours.size() == 0) break;
        }

        for (MatOfPoint cnt : contours){
            cnt.release();
        }

        return widest_contours;
    }

    public static int findLargestContourIndex(List<MatOfPoint> contours) {
        int index = 0;
        double maxArea = 0;
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint cnt = contours.get(i);
            double area = contourArea(cnt);
            if (area > maxArea) {
                maxArea = area;
                index = i;
            }
        }
        return index;
    }

    public static List<MatOfPoint> findNLargestContours(int n, List<MatOfPoint> contours) {
        List<MatOfPoint> new_contours = new ArrayList<>();

        for (int j = 0; j < n; j++) {
            int largest_index = findLargestContourIndex(contours);
            new_contours.add(contours.get(largest_index));

            contours.remove(largest_index);
            if (contours.size() == 0) break;
        }

        for (MatOfPoint cnt : contours){
            cnt.release();
        }

        return new_contours;
    }

}
