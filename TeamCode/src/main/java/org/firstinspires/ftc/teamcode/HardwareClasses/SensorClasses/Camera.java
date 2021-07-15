package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Ew.BlueAimPipeline;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Ew.RedAimPipeline;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Ew.RingFinderPipeline;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
	
	OpenCvWebcam webcam;
	public RingFinderPipeline ringFinderPipeline = new RingFinderPipeline();
	public BlueAimPipeline blueAimPipeline = new BlueAimPipeline();
	public RedAimPipeline redAimPipeline = new RedAimPipeline();
	public OpenCvPipeline currentPipeline = new OpenCvPipeline() { public Mat processFrame(Mat input) { return null; } };
	
	
	public Camera(OpenCvWebcam webcam){
		this.webcam = webcam;
	}
	
	public void setPipeline(OpenCvPipeline pipeline){
		currentPipeline = pipeline;
		webcam.setPipeline(pipeline);
	}
	
	public void optimizeView(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW); }
	
	public void optimizeEfficiency(){ webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY); }
	
	public void startVision(int width, int height) {
		webcam.openCameraDeviceAsync(() -> webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT));
	}
	
	public void stopVision(){ webcam.closeCameraDevice(); }
	
	
	public int startingStackCount(){ return ringFinderPipeline.getRingCount(); }
	
	public double highTowerError() {
		if (Sensors.alliance == Sensors.Alliance.BLUE) {
			if (currentPipeline != blueAimPipeline) setPipeline(blueAimPipeline);
			return blueAimPipeline.towerDegreeError();
		}else{
			if (currentPipeline != redAimPipeline) setPipeline(redAimPipeline);
			return redAimPipeline.towerDegreeError();
		}
	}
	
	public double midTowerError() {
		if (Sensors.alliance == Sensors.Alliance.BLUE) {
			if (currentPipeline != redAimPipeline) setPipeline(redAimPipeline);
			return redAimPipeline.towerDegreeError();
		}else{
			if (currentPipeline != blueAimPipeline) setPipeline(blueAimPipeline);
			return blueAimPipeline.towerDegreeError();
		}
	}
	
	public double highTowerDistance() {
		if (Sensors.alliance == Sensors.Alliance.BLUE) {
			if (currentPipeline != blueAimPipeline) setPipeline(blueAimPipeline);
			return blueAimPipeline.distance2Goal();
		} else {
			if (currentPipeline != redAimPipeline) setPipeline(redAimPipeline);
			return redAimPipeline.distance2Goal();
		}
	}
	
	
	public double midTowerDistance() {
		if (Sensors.alliance == Sensors.Alliance.BLUE) {
			if (currentPipeline != redAimPipeline) setPipeline(redAimPipeline);
			return redAimPipeline.distance2Goal();
		} else {
			if (currentPipeline != blueAimPipeline) setPipeline(blueAimPipeline);
			return blueAimPipeline.distance2Goal();
		}
	}
	
	
	public boolean isTowerFound(){ return blueAimPipeline.isTowerFound; }
	
	public double shooterOffsetAngle(){ return blueAimPipeline.shooterOffsetAngle(); }
	
	
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	public double leftPSAimAngle(){
		if (Sensors.alliance == Sensors.Alliance.BLUE) {
			if (currentPipeline != blueAimPipeline) setPipeline(blueAimPipeline);
			return Robot.closestTarget(blueAimPipeline.getPSDegreeError(BlueAimPipeline.PowerShot.LEFT));
		}else {
			if (currentPipeline != redAimPipeline) setPipeline(redAimPipeline);
			return Robot.closestTarget(redAimPipeline.getPSDegreeError(RedAimPipeline.PowerShot.LEFT));
		}
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	public double centerPSAimAngle(){
		if (Sensors.alliance == Sensors.Alliance.BLUE) {
			if (currentPipeline != blueAimPipeline) setPipeline(blueAimPipeline);
			return Robot.closestTarget(blueAimPipeline.getPSDegreeError(BlueAimPipeline.PowerShot.CENTER));
		}else {
			if (currentPipeline != redAimPipeline) setPipeline(redAimPipeline);
			return Robot.closestTarget(redAimPipeline.getPSDegreeError(RedAimPipeline.PowerShot.CENTER));
		}
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	public double rightPSAimAngle(){
		if (Sensors.alliance == Sensors.Alliance.BLUE) {
			if (currentPipeline != blueAimPipeline) setPipeline(blueAimPipeline);
			return Robot.closestTarget(blueAimPipeline.getPSDegreeError(BlueAimPipeline.PowerShot.RIGHT));
		}else {
			if (currentPipeline != redAimPipeline) setPipeline(redAimPipeline);
			return Robot.closestTarget(redAimPipeline.getPSDegreeError(RedAimPipeline.PowerShot.RIGHT));
		}
	}
	
}
