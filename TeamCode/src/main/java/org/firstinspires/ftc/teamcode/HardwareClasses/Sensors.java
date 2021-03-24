package org.firstinspires.ftc.teamcode.HardwareClasses;


import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Camera;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.DistanceSensor;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Gyro;
import org.firstinspires.ftc.utilities.IMU;
import org.openftc.easyopencv.OpenCvWebcam;

public class Sensors {
	
	public static Gyro gyro;
	public static Camera frontCamera, backCamera;
	public static DistanceSensor backDist, rightDist;

	public Sensors(IMU imu, OpenCvWebcam frontCamera, OpenCvWebcam backCamera, DistanceSensor backDist, DistanceSensor rightDist){
		Sensors.gyro = new Gyro(imu, 0);
		Sensors.frontCamera = new Camera(frontCamera);
		Sensors.backCamera = new Camera(backCamera);
		Sensors.backDist = backDist;
		Sensors.rightDist = rightDist;
	}
	
	public double XCoordinate(){
		return 2;
	}


}
