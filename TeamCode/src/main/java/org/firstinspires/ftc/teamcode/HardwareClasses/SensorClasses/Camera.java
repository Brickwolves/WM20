package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

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
	public StartingStackPipeline startingStackPipeline = new StartingStackPipeline();
	public TowerAimPipeline towerAimPipeline = new TowerAimPipeline();
	
	public Camera(OpenCvWebcam webcam){
		this.webcam = webcam;
	}
	
	public void setPipeline(OpenCvPipeline pipeline){
		webcam.setPipeline(pipeline);
	}
	
	public void optimizeView(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW); }
	
	public void optimizeEfficiency(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY); }
	
	public void startVision() {
		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			
			@Override
			public void onOpened() { webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); }
		});
	}
	
	public int startingStackAnalysis(){ return startingStackPipeline.getStackAnalysis(); }
	
	public int startingStackCount(){ return startingStackPipeline.getRingCount(); }
	
	public double towerAimError(){ return towerAimPipeline.getDegreeError(); }
	
	
}
