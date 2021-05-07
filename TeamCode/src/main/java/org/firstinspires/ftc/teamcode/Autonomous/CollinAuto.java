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
import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

@Autonomous(name = "Collin Auto", group = "Auto")
@Disabled
/*
it was the best of autos, it was the worst of autos. actually nevermind that philisohpical nonsense, it's just
the worst of autos. if you find this code i honestly recommend you delete it from the repository, erase it from
your computer, take a hammer to your hard drive, and then burn the remnants. there may be some stuff that's
salvageable but honestly it's 4:48 on a thursday and i have nothing else to do because this entire opmode
is hopeless so im writing this comment. hopefully this code will, after this weekend, be consigned to oblivion
in favour of newer auto code.
 */

//TODO nothing works fix it all (or maybe just destroy it and restart from the beginning)
public class CollinAuto extends OpMode {
	
	private final ElapsedTime feederTime = new ElapsedTime();
	private final ElapsedTime mainTime = new ElapsedTime();
	
	private Gyro gyro;
	private Robot robot;
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
		
	}
	
	public void init_loop(){
		intake.fabricRetract();
		intake.intakeOff();
		shooter.resetFeeder();
		shooter.lockFeeder();
	}
	
	public void start() {
		mainTime.reset();
		robot.resetGyro(0);
		robot.resetWithoutEncoders();
		shooter.newState(Shooter.ShooterState.POWER_SHOT);


	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void loop() {
		
		switch (currentMainState) {
			case STATE_DIAGONAL:
				robot.strafe(1000,0,78.311,1,0,0);
				if(robot.isStrafeComplete){
					newState(MainState.STATE_TURN1);
				}
				break;
			
			case STATE_TURN1:
				robot.turn(10, 1, .5);
				newState(MainState.STATE_SHOOT1);
				break;


			case STATE_SHOOT1:
				if (shooter.feederCount() >= 1) {
					newState(MainState.STATE_TURN2);
					break;
				}
				
				robot.setPower(0,0,0,0);
				shooter.powerShot();
				
				if(mainTime.seconds()>1){
					shooter.feederState(true);
				}
				break;
			
			case STATE_TURN2:
				robot.turn(-10, 1, 1);
				newState(MainState.STATE_SHOOT2);
				break;

			case STATE_SHOOT2:
				if (shooter.feederCount() >= 1) {
					newState(MainState.STATE_TURN3);
					break;
				}

				robot.setPower(0,0,0,0);
				shooter.powerShot();

				if(mainTime.seconds()>1){
					shooter.feederState(true);
				}
				break;

			case STATE_TURN3:
				robot.turn(-10, 1, 1);
				newState(MainState.STATE_SHOOT3);
				break;

			case STATE_SHOOT3:
				if (shooter.feederCount() >= 1) {
					newState(MainState.STATE_TURN4);
					break;
				}

				robot.setPower(0,0,0,0);
				shooter.powerShot();

				if(mainTime.seconds()>1){
					shooter.feederState(true);
				}
				break;

			case STATE_TURN4:
				shooter.shooterOff();
				robot.turn(10, 1, 1);
				newState(MainState.STATE_FORWARD);

			case STATE_FORWARD:
				robot.strafe(org.firstinspires.ftc.teamcode.Autonomous.AutoUtils.Utils.convertInches2Ticks(6),0,0,1,0,0);
				if(robot.isStrafeComplete){
					newState(MainState.STATE_OFF);
				}
				break;

			case STATE_OFF:
				//off

		}
		
		telemetry.addData("flywheel rpm = ", Math.abs(shooter.updateRPM()));
		
		
	}
	

	
	
	private void newState(MainState newState) {
		currentMainState = newState;
		mainTime.reset();
	}
	
	private enum MainState {
		STATE_DIAGONAL,
		STATE_TURN1,
		STATE_SHOOT1,
		STATE_TURN2,
		STATE_SHOOT2,
		STATE_TURN3,
		STATE_SHOOT3,
		STATE_TURN4,
		STATE_FORWARD,
		STATE_OFF
	}
}
