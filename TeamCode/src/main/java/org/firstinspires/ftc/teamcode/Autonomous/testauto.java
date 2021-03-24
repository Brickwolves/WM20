package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Gyro;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumChassis;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

@Autonomous(name = "testauto", group = "Auto")
@Disabled
public class testauto extends OpMode {
	
	private final ElapsedTime feederTime = new ElapsedTime();
	private final ElapsedTime mainTime = new ElapsedTime();
	
	private Gyro gyro;
	private MecanumChassis robot;
	private Shooter shooter;
	private Intake intake;
	boolean isFeederLocked = true;
	
	private MainState currentMainState = MainState.STATE_STRAFE;
	
	
	@Override
	public void init() {
		Servo feeder = hardwareMap.get(Servo.class, "feeder");
		Servo feederLock = hardwareMap.get(Servo.class, "feederlock");
		
		Servo outerRollerOne = hardwareMap.get(Servo.class, "outerrollerone");
		Servo outerRollerTwo = hardwareMap.get(Servo.class, "outerrollertwo");
		
		DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
		DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontright");
		DcMotor backLeft = hardwareMap.get(DcMotor.class, "backleft");
		DcMotor backRight = hardwareMap.get(DcMotor.class, "backright");
		
		DcMotor shooterOne = hardwareMap.get(DcMotor.class, "shooterone");
		DcMotor shooterTwo = hardwareMap.get(DcMotor.class, "shootertwo");
		DcMotor intakeDrive = hardwareMap.get(DcMotor.class, "intake");
		
		Utils.setHardwareMap(hardwareMap);
		IMU imu = new IMU("imu");
		gyro = new Gyro(imu, 0);
		shooter = new Shooter(shooterOne, shooterTwo, feeder, feederLock);
		intake = new Intake(intakeDrive, outerRollerOne,outerRollerTwo);
		robot = new MecanumChassis(frontLeft, frontRight, backLeft, backRight);
	}
	
	public void init_loop(){
		intake.retractBumper();
		intake.intakeOff();
		shooter.resetFeeder();
		shooter.lockFeeder();
		telemetry.addData("cosine: ", Math.cos(-90));
		telemetry.addData("sine: ", Math.sin(-90));
	}
	
	public void start() {
		mainTime.reset();
		robot.resetGyro(0);
		robot.resetWithoutEncoders();
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void loop() {
		
		switch (currentMainState) {
			case STATE_TURN:
				robot.turn(90,.8, 0.1);
				telemetry.addData("state = ", "turn");
				if(robot.isTurnComplete){
					newState(MainState.STATE_STRAFE);
				}
				break;
			
			case STATE_STRAFE:
				robot.strafe(2000,90,0,1,0,0);
				telemetry.addData("state = ", "strafe");
				telemetry.addData("strafe drive = ", robot.strafeDrive);
				if(robot.isStrafeComplete){
					newState(MainState.STATE_SHOOT);
				}
				break;
			
			case STATE_SHOOT:
				robot.setPowerAuto(0,0,90,0);
				telemetry.addData("state = ", "off");
				
		}
		
		telemetry.addData("flywheel rpm = ", Math.abs(shooter.updateRPM()));
		telemetry.update();
		
		
	}
	

	
	
	private void newState(MainState newState) {
		currentMainState = newState;
		mainTime.reset();
	}
	
	private enum MainState {
		STATE_TURN,
		STATE_STRAFE,
		STATE_SHOOT
	}
}
