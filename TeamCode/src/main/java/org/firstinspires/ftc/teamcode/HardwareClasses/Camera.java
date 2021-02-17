package org.firstinspires.ftc.teamcode.HardwareClasses;

import org.firstinspires.ftc.teamcode.Vision.EasyOpenCVExample;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
	
	OpenCvWebcam webcam;
	RingDetectingPipeline pipeline = new RingDetectingPipeline();
	
	
	public Camera(OpenCvWebcam webcam){
		webcam.setPipeline(pipeline);
		this.webcam = webcam;
	}
	
	public void optimizeView(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW); }
	
	public void optimizeEfficiency(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY); }
	
	public void openCamera() {
		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			
			@Override
			public void onOpened() { webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN); }
		});
	}
	
	public int getAnalysis(){ return pipeline.getAnalysis(); }
	
	public RingDetectingPipeline.RingCount getRingCount(){ return pipeline.ringCount; }
	
	
	public static class RingDetectingPipeline extends OpenCvPipeline
	{
		public enum RingCount
		{
			FOUR,
			ONE,
			NONE
		}
		
		static final Scalar BLUE = new Scalar(0, 0, 255);
		static final Scalar GREEN = new Scalar(0, 255, 0);
		
		
		
		static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);
		
		static final int REGION_WIDTH = 35;
		static final int REGION_HEIGHT = 25;
		
		final int FOUR_RING_THRESHOLD = 150;
		final int ONE_RING_THRESHOLD = 135;
		
		Point region1_pointA = new Point(
				REGION1_TOPLEFT_ANCHOR_POINT.x,
				REGION1_TOPLEFT_ANCHOR_POINT.y);
		Point region1_pointB = new Point(
				REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
				REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
		
		
		Mat region1_Cb;
		Mat YCrCb = new Mat();
		Mat Cb = new Mat();
		int avg1;
		
		
		private volatile RingCount ringCount = RingCount.FOUR;
		
		
		void inputToCb(Mat input)
		{
			Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
			Core.extractChannel(YCrCb, Cb, 1);
		}
		
		@Override
		public void init(Mat firstFrame)
		{
			inputToCb(firstFrame);
			
			region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
		}
		
		@Override
		public Mat processFrame(Mat input)
		{
			inputToCb(input);
			
			avg1 = (int) Core.mean(region1_Cb).val[0];
			
			Imgproc.rectangle(
					input, // Buffer to draw on
					region1_pointA, // First point which defines the rectangle
					region1_pointB, // Second point which defines the rectangle
					BLUE, // The color the rectangle is drawn in
					2); // Thickness of the rectangle lines
			
			ringCount = RingCount.FOUR; // Record our analysis
			if(avg1 > FOUR_RING_THRESHOLD){
				ringCount = RingCount.FOUR;
			}else if (avg1 > ONE_RING_THRESHOLD){
				ringCount = RingCount.ONE;
			}else{
				ringCount = RingCount.NONE;
			}
			
			Imgproc.rectangle(
					input, // Buffer to draw on
					region1_pointA, // First point which defines the rectangle
					region1_pointB, // Second point which defines the rectangle
					GREEN, // The color the rectangle is drawn in
					-1); // Negative thickness means solid fill
			
			return input;
		}
		
		public int getAnalysis()
		{
			return avg1;
		}
	}
	
}
