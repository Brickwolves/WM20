package org.firstinspires.ftc.teamcode.HardwareClasses;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Camera;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.MaxSonarI2CXL;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Gyro;
import org.firstinspires.ftc.utilities.IMU;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import static org.firstinspires.ftc.utilities.Utils.hardwareMap;

public class Sensors {
	
	
	public static Gyro gyro;
	public static Camera frontCamera, backCamera;
	private static long currentTimeMilis;
	
	
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
		currentTimeMilis = System.currentTimeMillis();
		gyro.update();
	}
	
	public static long currentTimeMilis(){ return currentTimeMilis; }


}
