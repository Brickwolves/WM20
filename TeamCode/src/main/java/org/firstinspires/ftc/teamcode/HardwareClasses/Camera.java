package org.firstinspires.ftc.teamcode.HardwareClasses;

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
			public void onOpened() { webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN); }
		});
	}
	
	public int getStackAnalysis(){ return pipeline.getStackAnalysis(); }
	
	public int getRingCount(){ return pipeline.ringCount; }
	
	
	public static class RingDetectingPipeline extends OpenCvPipeline {
		
		private int ringCount;
		int avgStack;
		
		static final Scalar BLUE = new Scalar(0, 0, 255);
		static final Scalar GREEN = new Scalar(0, 255, 0);
		
		
		static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(160,98);
		static final int REGION1_WIDTH = 35;
		static final int REGION1_HEIGHT = 45;
		
		final int FOUR_RING_THRESHOLD = 143;
		final int ONE_RING_THRESHOLD = 133;
		
		Point region1_pointA = new Point(
				REGION1_TOPLEFT_ANCHOR_POINT.x,
				REGION1_TOPLEFT_ANCHOR_POINT.y);
		Point region1_pointB = new Point(
				REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH,
				REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);
		
		
		Mat region1_Cb;
		Mat YCrCb = new Mat();
		Mat Cb = new Mat();
		
		void inputToCb(Mat input) {
			Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
			Core.extractChannel(YCrCb, Cb, 1);
		}
		
		@Override
		public void init(Mat firstFrame) {
			
			inputToCb(firstFrame);
			region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
		}
		
		@Override
		public Mat processFrame(Mat input) {
			inputToCb(input);
			
			avgStack = (int) Core.mean(region1_Cb).val[0];
			
			Imgproc.rectangle(
					input, // Buffer to draw on
					region1_pointA, // First point which defines the rectangle
					region1_pointB, // Second point which defines the rectangle
					BLUE, // The color the rectangle is drawn in
					3); // Thickness of the rectangle lines
			
			
			ringCount = 4; // Record our analysis
			if(avgStack > FOUR_RING_THRESHOLD){
				ringCount = 4;
			}else if (avgStack > ONE_RING_THRESHOLD){
				ringCount = 1;
			}else{
				ringCount = 0;
			}
			
			return input;
		}
		
		public int getStackAnalysis() { return avgStack; }
	}
	
}
