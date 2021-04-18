package org.firstinspires.ftc.teamcode.HardwareClasses;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Camera;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.DistanceSensor;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Gyro;
import org.firstinspires.ftc.utilities.IMU;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import static org.firstinspires.ftc.utilities.Utils.getHardwareMap;

public class Sensors {
	
	private static final int cameraMonitorViewId = getHardwareMap().appContext.getResources().getIdentifier(
			"cameraMonitorViewId", "id", getHardwareMap().appContext.getPackageName());
	private static final WebcamName frontCamName = getHardwareMap().get(WebcamName.class, "Front Camera");
	private static final OpenCvWebcam frontWebcam = OpenCvCameraFactory.getInstance().createWebcam(frontCamName, cameraMonitorViewId);
	
	private static final WebcamName backCamName = getHardwareMap().get(WebcamName.class, "Back Camera");
	private static final OpenCvWebcam backWebcam = OpenCvCameraFactory.getInstance().createWebcam(backCamName, cameraMonitorViewId);
	
	private static final IMU imu = new IMU("imu");
	
	public static Gyro gyro;
	public static Camera frontCamera, backCamera;
	public static DistanceSensor backDist, rightDist;
	
	
	public static void init(){
		Sensors.gyro = new Gyro(imu, 0);
		Sensors.frontCamera = new Camera(frontWebcam);
		Sensors.backCamera = new Camera(backWebcam);
		//Sensors.backDist = backDist;
		//Sensors.rightDist = rightDist;
	}
	
	public static void update(){
		gyro.update();
	}


}
