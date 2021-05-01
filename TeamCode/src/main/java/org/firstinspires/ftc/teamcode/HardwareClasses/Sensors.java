package org.firstinspires.ftc.teamcode.HardwareClasses;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Camera;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Gyro;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.MathUtils;
import org.firstinspires.ftc.utilities.RingBufferOwen;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static org.firstinspires.ftc.utilities.Utils.hardwareMap;

public class Sensors {
	
	
	public static Gyro gyro;
	public static Camera frontCamera, backCamera;
	private static long currentTimeMillis;
	
	static RingBufferOwen timeRing = new RingBufferOwen(3);
	static RingBufferOwen frRing = new RingBufferOwen(3);
	static RingBufferOwen flRing = new RingBufferOwen(3);
	static RingBufferOwen brRing = new RingBufferOwen(3);
	static RingBufferOwen blRing = new RingBufferOwen(3);
	private static double frRPM, flRPM, brRPM, blRPM;
	
	
	public static void init(){
		IMU imu = new IMU("imu");
		
		int cameraMonitorViewId = hardwareMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap().appContext.getPackageName());
		
		WebcamName frontCamName = hardwareMap().get(WebcamName.class, "Front Camera");
		OpenCvWebcam frontWebcam = OpenCvCameraFactory.getInstance().createWebcam(frontCamName, cameraMonitorViewId);
		
		WebcamName backCamName = hardwareMap().get(WebcamName.class, "Back Camera");
		OpenCvWebcam backWebcam = OpenCvCameraFactory.getInstance().createWebcam(backCamName);
		
		Sensors.gyro = new Gyro(imu, 0);
		Sensors.frontCamera = new Camera(frontWebcam);
		Sensors.backCamera = new Camera(backWebcam);
	}
	
	public static void update(){
		currentTimeMillis = System.currentTimeMillis();
		gyro.update();
		
		long deltaMili = currentTimeMillis - timeRing.getValue(currentTimeMillis);
		double deltaMinutes = deltaMili / 60000.0;
		
		long frPosition = Robot.frontRight.getCurrentPosition();
		double frDeltaRotations = (frPosition - frRing.getValue(frPosition)) / 537.7;
		frRPM = frDeltaRotations / deltaMinutes;
		
		long flPosition = Robot.frontLeft.getCurrentPosition();
		double flDeltaRotations = (flPosition - flRing.getValue(flPosition)) / 537.7;
		flRPM = flDeltaRotations / deltaMinutes;
		
		long brPosition = Robot.backRight.getCurrentPosition();
		double brDeltaRotations = (brPosition - brRing.getValue(brPosition)) / 537.7;
		brRPM = brDeltaRotations / deltaMinutes;
		
		long blPosition = Robot.backLeft.getCurrentPosition();
		double blDeltaRotations = (blPosition - blRing.getValue(blPosition)) / 537.7;
		blRPM = blDeltaRotations / deltaMinutes;
	}
	
	public static long currentTimeMilis(){ return currentTimeMillis; }


	
	//ROBOT MOVEMENT
	public static double robotVelocityComponent(double angle){
		double drive = (frRPM + flRPM + brRPM + blRPM) / 4;
		double strafe = (frRPM - flRPM - brRPM + blRPM) / 4;
		double velocityAngle;
		
		double velocity = Math.sqrt(Math.pow(drive, 2) + Math.pow(strafe, 2));
		
		if (velocity == 0 ) velocityAngle = 0;
		else velocityAngle = - MathUtils.degATan(drive, strafe) + 90;
		
		angle = angle - velocityAngle;
		
		return MathUtils.degCos(angle) * velocity;
	}
	
	public static double maxRobotRPM(){
		return max(max(abs(frRPM), abs(flRPM)), max(abs(brRPM), abs(blRPM)));
	}
	
	public static boolean isRobotMoving(){
		return maxRobotRPM() < 70 && Robot.drive < .4 && Robot.strafe < .4 && Robot.turn < .2;
	}
	
	
}
