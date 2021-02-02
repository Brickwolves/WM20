package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Gyro;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumChassis;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

@Autonomous(name = "Owen Kinky Auto", group = "Auto")

public class OwenKinkyAuto extends OpMode {
	
	private final ElapsedTime feederTime = new ElapsedTime();
	private final ElapsedTime mainTime = new ElapsedTime();
	
	private Gyro gyro;
	private MecanumChassis robot;
	private Shooter shooter;
	private Intake intake;
	boolean isFeederLocked = true;
	
	private MainState currentMainState = MainState.STATE_FORWARD;
	
	
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
		robot = new MecanumChassis(frontLeft, frontRight, backLeft, backRight, gyro);
	}
	
	public void init_loop(){
		intake.retractReach();
		intake.intakeOff();
		shooter.resetFeeder();
		shooter.lockFeeder();
	}
	
	public void start() {
		mainTime.reset();
		robot.resetGyro();
		robot.resetMotors();
		shooter.newState(Shooter.ShooterState.STATE_TOP_GOAL);
		
		
	}
	
	@Override
	public void loop() {
		
		switch (currentMainState) {
			case STATE_FORWARD:
				robot.strafe(2000,0,0,1,0,0,1);
				intake.deployReach();
				
				if(robot.currentTicks>750) {
					
					intake.intakeOn();
				}
				if(robot.isStrafeFinished){
					newState(MainState.STATE_LINEUP);
				}
				break;
			
			case STATE_LINEUP:
				robot.strafe(400,0,180,.5,0,0,2);
				if(robot.currentTicks < 350){ intake.intakeOn(); }
				else{ intake.intakeOff(); intake.retractReach(); }
				
				if(robot.isStrafeFinished){
					newState(MainState.STATE_SHOOT);
				}
				break;
				
			case STATE_SHOOT:
				if (shooter.feederCount() >= 3) {
					newState(MainState.STATE_STRAFE);
					break;
				}
				
				robot.setPower(0,0,0,0);
				intake.retractReach();
				intake.intakeOff();
				shooter.topGoal();
				
				if(mainTime.seconds()>1){
					shooter.feederState(true);
				}
				break;
			
			case STATE_STRAFE:
				shooter.shooterOff();
		}
		
		telemetry.addData("flywheel rpm = ", Math.abs(shooter.updateRPM()));
		
		
	}
	

	
	
	private void newState(MainState newState) {
		currentMainState = newState;
		mainTime.reset();
	}
	
	private enum MainState {
		STATE_FORWARD,
		STATE_LINEUP,
		STATE_SHOOT,
		STATE_STRAFE,
	}
}
