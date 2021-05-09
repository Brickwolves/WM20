package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
	
	OpenCvWebcam webcam;
	public RingFinderPipeline ringFinderPipeline = new RingFinderPipeline();
	public AutoAimPipeline autoAimPipeline = new AutoAimPipeline();
	
	public Camera(OpenCvWebcam webcam){
		this.webcam = webcam;
	}
	
	public void setPipeline(OpenCvPipeline pipeline){
		webcam.setPipeline(pipeline);
	}
	
	public void optimizeView(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW); }
	
	public void optimizeEfficiency(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY); }
	
	public void startVision(int width, int height) {
		webcam.openCameraDeviceAsync(() -> webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT));
	}
	
	public void stopVision(){ webcam.closeCameraDevice(); }
	
	
	public int startingStackCount(){ return ringFinderPipeline.getRingCount(); }
	
	
	public double towerAimError(){ return autoAimPipeline.getDegreeError(); }
	
	public double towerDistance(){ return autoAimPipeline.distance2Goal(); }
	
	public boolean isTowerFound(){ return autoAimPipeline.isTowerFound; }
	
	public double shooterOffsetAngle(){ return autoAimPipeline.shooterOffsetAngle(); }
	
	
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	public double leftPSAimAngle(){ return Robot.closestTarget(autoAimPipeline.getPSDegreeError(AutoAimPipeline.PowerShot.PS_LEFT)); }
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	public double centerPSAimAngle(){  return Robot.closestTarget(autoAimPipeline.getPSDegreeError(AutoAimPipeline.PowerShot.PS_CENTER)); }
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	public double rightPSAimAngle(){  return Robot.closestTarget(autoAimPipeline.getPSDegreeError(AutoAimPipeline.PowerShot.PS_RIGHT)); }
	
}
