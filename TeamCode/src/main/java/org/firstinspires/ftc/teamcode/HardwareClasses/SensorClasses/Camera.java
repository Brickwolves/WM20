package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
	
	OpenCvWebcam webcam;
	public StartingStackPipeline startingStackPipeline = new StartingStackPipeline();
	public RingFinderPipeline ringFinderPipeline = new RingFinderPipeline();
	public TowerAimPipeline towerAimPipeline = new TowerAimPipeline();
	public PSAimPipeline PSAimPipeline = new PSAimPipeline();
	public OpenCvPipeline currentPipeline;
	
	public Camera(OpenCvWebcam webcam){
		this.webcam = webcam;
	}
	
	public void setPipeline(OpenCvPipeline pipeline){
		webcam.setPipeline(pipeline);
		OpenCvPipeline currentPipeline = pipeline;
	}
	
	public void optimizeView(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW); }
	
	public void optimizeEfficiency(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY); }
	
	public void startVision(int width, int height) {
		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			
			@Override
			public void onOpened() { webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT); }
		});
	}
	
	public int startingStackAnalysis(){ return startingStackPipeline.getStackAnalysis(); }
	
	public int startingStackCount(){ return ringFinderPipeline.getRingCount(); }
	
	public double towerAimError(){ return towerAimPipeline.getDegreeError(); }
	
	public double towerDistance(){ return towerAimPipeline.distance2Goal(); }
	
	public boolean isTowerFound(){ return towerAimPipeline.isTowerFound; }
	
	public double leftPSAimError(){ return PSAimPipeline.getPSError(org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.PSAimPipeline.PS.LEFT); }
	
	public double centerPSAimError(){ return PSAimPipeline.getPSError(org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.PSAimPipeline.PS.CENTER); }
	
	public double rightPSAimError(){ return PSAimPipeline.getPSError(org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.PSAimPipeline.PS.RIGHT); }
	
	public boolean arePSFound(){ return PSAimPipeline.arePSFound; }
	
	
}
