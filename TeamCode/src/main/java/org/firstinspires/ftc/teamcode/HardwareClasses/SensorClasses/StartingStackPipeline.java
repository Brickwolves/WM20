package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class StartingStackPipeline extends OpenCvPipeline {
	
	private int ringCount;
	int avgBottom;
	int avgTop;
	
	static final Scalar BLUE = new Scalar(0, 0, 255);
	static final Scalar GREEN = new Scalar(0, 255, 0);
	
	
	static final Point TOPLEFT_ANCHOR_POINT = new Point(1500,560);
	static final int BOTTOM_WIDTH = 150;
	static final int BOTTOM_HEIGHT = 250;
	
	final int ONE_RING_THRESHOLD = 129;
	final int FOUR_RING_THRESHOLD = 142;
	
	Point pointA = new Point(
			TOPLEFT_ANCHOR_POINT.x,
			TOPLEFT_ANCHOR_POINT.y);
	Point bottom_pointB = new Point(
			TOPLEFT_ANCHOR_POINT.x + BOTTOM_WIDTH,
			TOPLEFT_ANCHOR_POINT.y + BOTTOM_HEIGHT);
	
	
	Mat bottomRegion_Cb;
	Mat topRegion_Cb;
	Mat YCrCb = new Mat();
	Mat Cb = new Mat();
	
	void inputToCb(Mat input) {
		Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
		Core.extractChannel(YCrCb, Cb, 1);
	}
	
	@Override
	public void init(Mat firstFrame) {
		
		inputToCb(firstFrame);
		bottomRegion_Cb = Cb.submat(new Rect(pointA, bottom_pointB));
	}
	
	@Override
	public Mat processFrame(Mat input) {
		inputToCb(input);
		
		avgBottom = (int) Core.mean(bottomRegion_Cb).val[0];
		
		Imgproc.rectangle(
				input, // Buffer to draw on
				pointA, // First point which defines the rectangle
				bottom_pointB, // Second point which defines the rectangle
				BLUE, // The color the rectangle is drawn in
				12); // Thickness of the rectangle lines
		
		
		ringCount = 4; // Record our analysis
		if(avgBottom > FOUR_RING_THRESHOLD){
			ringCount = 4;
		}else if (avgBottom > ONE_RING_THRESHOLD){
			ringCount = 1;
		}else{
			ringCount = 0;
		}
		
		return input;
	}
	
	public int getStackAnalysis() { return avgBottom; }
	
	public int getRingCount() { return ringCount; }
}