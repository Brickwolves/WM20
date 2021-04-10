package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.Katana;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumChassis;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.WobbleGripper;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import static android.os.SystemClock.sleep;

@Autonomous(name = "REAL AUTONOMOUS", group = "Auto")

public class RealAutonomous extends OpMode {
	
	private Controller operator;
	private MecanumChassis robot;
	private Shooter shooter;
	private Intake intake;
	private WobbleGripper wobble;
	private Katana katana;
	private static Sensors sensors;
	
	
	private MainState currentMainState = MainState.state1Drive;
	private final ElapsedTime mainTime = new ElapsedTime();
	

	private static double ringCount = 0;
	private final boolean ringsFound = false;

	@Override
	public void init() {
		Servo feeder = hardwareMap.get(Servo.class, "feeder");
		Servo feederLock = hardwareMap.get(Servo.class, "feederlock");
		
		Servo bumperLeft = hardwareMap.get(Servo.class, "bumperleft");
		Servo bumperRight = hardwareMap.get(Servo.class, "bumperright");
		
		Servo lifter = hardwareMap.get(Servo.class, "lifter");
		Servo gripperOne = hardwareMap.get(Servo.class, "gripperone");
		Servo gripperTwo = hardwareMap.get(Servo.class, "grippertwo");
		
		Servo katanaLeft = hardwareMap.get(Servo.class, "katanaleft");
		Servo katanaRight = hardwareMap.get(Servo.class, "katanaright");
		
		DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
		DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontright");
		DcMotor backLeft = hardwareMap.get(DcMotor.class, "backleft");
		DcMotor backRight = hardwareMap.get(DcMotor.class, "backright");
		
		DcMotor shooterOne = hardwareMap.get(DcMotor.class, "shooterone");
		DcMotor shooterTwo = hardwareMap.get(DcMotor.class, "shootertwo");
		DcMotor intakeDrive = hardwareMap.get(DcMotor.class, "intake");
		
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		WebcamName backCamName = hardwareMap.get(WebcamName.class, "Back Camera");
		OpenCvWebcam backCam = OpenCvCameraFactory.getInstance().createWebcam(backCamName, cameraMonitorViewId);
		
		//cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		//WebcamName frontCamName = hardwareMap.get(WebcamName.class, "Front Camera");
		//OpenCvWebcam frontCam = OpenCvCameraFactory.getInstance().createWebcam(frontCamName, cameraMonitorViewId);
		
		Utils.setHardwareMap(hardwareMap);
		IMU imu = new IMU("imu");
		
		operator = new Controller(gamepad2);
		shooter = new Shooter(shooterOne, shooterTwo, feeder, feederLock);
		intake = new Intake(intakeDrive, bumperLeft, bumperRight);
		robot = new MecanumChassis(frontLeft, frontRight, backLeft, backRight);
		wobble = new WobbleGripper(gripperOne, gripperTwo, lifter);
		katana = new Katana(katanaLeft, katanaRight);
		sensors = new Sensors(imu, null, backCam, null, null);
		
		Sensors.backCamera.setPipeline(Sensors.backCamera.ringFinderPipeline);
		Sensors.backCamera.startVision(1920, 1080);
	}
	
	public void init_loop(){
		operator.update();
		shooter.resetFeeder();
		shooter.lockFeeder();
		
		if(operator.RBToggle()) wobble.gripperGrip();
		else wobble.gripperHalf();
		
		if(operator.crossToggle()){
			intake.intakeStallControl();
			intake.setBumperThreshold(1);
		}else{
			intake.intakeOff();
			intake.retractBumper();
		}
		
		if(operator.upPress()) Sensors.backCamera.setPipeline(Sensors.backCamera.ringFinderPipeline);
		if(operator.downPress()) Sensors.backCamera.setPipeline(Sensors.backCamera.startingStackPipeline);
		
		telemetry.addData("Ring Count = ", Sensors.backCamera.startingStackCount());
		telemetry.addData("current pipeline", Sensors.backCamera.startingStackAnalysis());
		telemetry.update();
		
		sleep(50);
	}
	
	public void start() {
		mainTime.reset();
		katana.katanaHalfFold();
		robot.resetGyro(180);
		robot.resetWithoutEncoders();
		ringCount = Sensors.backCamera.startingStackCount();
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void loop() {
		
		telemetry.addData("current state: ", currentMainState);
		
		switch ((int) ringCount) {
			case 0:
				switch (currentMainState) {
					//drive to target A
					case state1Drive:
						robot.strafe(33.5, 180, 0, 1, 0, 0);
						
						if (robot.currentInches > 18) wobble.armDown();
						
						if (robot.isStrafeComplete) {
							robot.setPower(0, 0, 0, 1);
							newState(MainState.state2WobbleGoal);
						}
						
						break;
					
					//put down wobble goal and back up
					case state2WobbleGoal:
						wobble.armDown();
						if (mainTime.seconds() > .9) {
							wobble.gripperOpen();
							robot.strafe(6, 180, 180, .5, 0, 0);
						} else robot.setPower(0, 0, 0, 1);
						
						
						if (mainTime.seconds() > .9 && robot.isStrafeComplete) {
							newState(MainState.state3Turn);
							wobble.armFold();
							wobble.gripperHalf();
						}
						break;
					
					
					//turn towards power shot shooting position
					case state3Turn: //turn
						robot.turn(270, 1, 0);
						
						if (robot.isTurnComplete) newState(MainState.state4Drive);
						
						break;
					
					
					//drive to power shot shooting position
					case state4Drive:
						robot.strafe(29.5, 270, 270, 1, 0, 0);
						
						if (robot.isStrafeComplete) newState(MainState.state5Turn);
						
						break;
					
					
					//turn to face powershots
					case state5Turn:
						if (mainTime.seconds() > .7) {
							robot.turn(-5, 1, 0);
							shooter.powerShot();
						}
						
						if (mainTime.seconds() > .7 && robot.isTurnComplete) newState(MainState.state6PS1);
						break;
					
					
					case state6PS1:
						shooter.powerShot();
						
						if (Shooter.feederCount() < 1) {
							robot.setPowerAuto(0, 0, 0);
							if (mainTime.seconds() > .8) shooter.feederState(true);
						} else {
							robot.turn(6, 1, 1);
							
							if (mainTime.seconds() > 1.7 && robot.isTurnComplete) newState(MainState.state7PS2);
							
						}
						break;
					
					
					//shoot power shot 2, turn to power shot 3
					case state7PS2:
						shooter.powerShot();
						if (Shooter.feederCount() < 2) {
							robot.setPowerAuto(0, 0, 6);
							if (mainTime.seconds() > .2) shooter.feederState(true);
							
						} else {
							robot.turn(11.3, 1, 1);
							
							if (robot.isTurnComplete) newState(MainState.state8PS3);
							
						}
						break;
					
					
					//shoot power shot 3, turn to second wobble goal
					case state8PS3:
						shooter.powerShot();
						wobble.armDown();
						wobble.gripperOpen();
						
						if (Shooter.feederCount() < 3) {
							if (mainTime.seconds() > 0) shooter.feederState(true);
							robot.setPowerAuto(0, 0, 11.3);
						} else {
							robot.turn(-37, 1, .8);
							shooter.shooterOff();
							
							if (robot.isTurnComplete) newState(MainState.state9Drive);
						}
						break;
					
					
					//drive to second wobble goal
					case state9Drive:
						robot.strafe(14, -37, 143, 1, 0, 0);
						if (robot.isStrafeComplete) {
							newState(MainState.state10WobbleGoal);
							robot.setPower(0, 0, 0, 1);
						}
						break;
					
					
					//pick up second wobble goal, turn away from target A
					case state10WobbleGoal:
						wobble.armDown();
						
						if (mainTime.seconds() > .8) wobble.gripperGrip();
						
						if (mainTime.seconds() > 1.6) {
							robot.turn(187, 1, 0);
							wobble.armPosition(.15);
							if (robot.isTurnComplete) newState(MainState.state11Drive);
						}
						break;
					
					
					//drive to target A
					case state11Drive:
						wobble.armPosition(.15);
						wobble.gripperGrip();
						robot.strafe(22, 187, 7, 1, 0, 0);
						
						if (robot.isStrafeComplete) newState(MainState.state13Turn);
						break;
						
					
					case state13Turn:
						wobble.gripperGrip();
						wobble.armPosition(.15);
						if (mainTime.seconds() > .7) robot.turn(-85, 1, .5);
						if (mainTime.seconds() > 1 && robot.isTurnComplete) newState(MainState.state12WobbleGoal);
						break;
						
						
					//put down second wobble goal and back up
					case state12WobbleGoal:
						wobble.armPosition(.15);
						wobble.gripperGrip();
						robot.strafe(10, -90, 90, .6, .2, 0);
						if(robot.isStrafeComplete)newState(MainState.stateFinished);
						break;
						
					case stateFinished:
						robot.setPowerAuto(0,0,-90,2);
				}
				
				break;
			
			case 1:
				switch (currentMainState) {
					//drive to WP1
					case state1Drive:
						robot.strafe(40, 180, 0, 1, .2, 0);
						
						if (robot.isStrafeComplete) newState(MainState.state1Turn);
						break;
					
					
					case state1Turn:
						if (mainTime.seconds() > .7) robot.turn(135, 1, .5);
						if (mainTime.seconds() > .9 && robot.isTurnComplete) newState(MainState.state1Diagonal);
						break;
					
					
					case state1Diagonal:
						robot.strafe(8, 135, -45, 1, .2, 0);
						if (robot.currentInches > 1) wobble.armDown();
						if (robot.isStrafeComplete) newState(MainState.state2WobbleGoal);
						break;
					
					
					//put down wobble goal and back up
					case state2WobbleGoal:
						wobble.armDown();
						if (mainTime.seconds() > .9) {
							newState(MainState.state3Turn);
							wobble.armFold();
							wobble.gripperOpen();
						}
						break;
					
					
					//turn to power shot shooting position
					case state3Turn:
						robot.turn(-127, .8, .4);
						
						if (robot.isTurnComplete) {
							newState(MainState.state4Drive);
							wobble.gripperHalf();
						}
						break;
					
					
					//drive to power shot shooting position
					case state4Drive:
						robot.strafe(22.5, -126, -126, 1, 0, 0);
						if (robot.isStrafeComplete) newState(MainState.state5Turn);
						break;
					
					
					//turn towards power shots
					case state5Turn:
						if (mainTime.seconds() > .8) {
							robot.turn(-4, 1, 0);
							shooter.powerShot();
						}
						
						if (mainTime.seconds() > .8 && robot.isTurnComplete) newState(MainState.state6PS1);
						break;
					
					
					case state6PS1:
						shooter.powerShot();
						
						if (Shooter.feederCount() < 1) {
							robot.setPowerAuto(0, 0, 2);
							if (mainTime.seconds() > .8) shooter.feederState(true);
						} else {
							robot.turn(7, 1, 1);
							if (mainTime.seconds() > 1.7 && robot.isTurnComplete) newState(MainState.state7PS2);
						}
						break;
					
					
					//shoot power shot 2, turn to power shot 3
					case state7PS2:
						shooter.powerShot();
						if (Shooter.feederCount() < 2) {
							robot.setPowerAuto(0, 0, 7);
							if (mainTime.seconds() > .2) shooter.feederState(true);
						} else {
							robot.turn(13, 1, 1);
							if (robot.isTurnComplete) newState(MainState.state8PS3);
						}
						break;
					
					
					//shoot power shot 3, turn to second wobble goal
					case state8PS3:
						shooter.powerShot();
						wobble.armDown();
						wobble.gripperOpen();
						
						if (Shooter.feederCount() < 3) {
							if (mainTime.seconds() > 0) shooter.feederState(true);
							robot.setPowerAuto(0, 0, 13);
						} else {
							robot.turn(-37, 1, .8);
							shooter.shooterOff();
							if (robot.isTurnComplete) newState(MainState.state9Drive);
						}
						break;
					
					//drive to second wobblee goal
					case state9Drive:
						robot.strafe(18, -37, 143, 1, 0, 0);
						if (robot.isStrafeComplete) {
							newState(MainState.state10WobbleGoal);
							robot.setPower(0, 0, 0, 1);
						}
						break;
					
					
					//pick up second wobble goal
					case state10WobbleGoal:
						wobble.armDown();
						if (mainTime.seconds() > 1) wobble.gripperGrip();
						
						if (mainTime.seconds() > 1.6) {
							wobble.armTele();
							robot.strafe(7, -37, 143, 1, 0, 0);
							
							if (robot.isStrafeComplete) {
								newState(MainState.state11Turn);
								robot.setPower(0, 0, 0, 1);
							}
						}
						break;
					
					
					//turn to face back wall
					case state11Turn:
						robot.turn(0, 1, 1);
						intake.setBumperThreshold(1);
						
						if (robot.isTurnComplete) {
							newState(MainState.state12Drive);
							robot.setPower(0, 0, 0, 1);
						}
						break;
					
					
					//strafe to line up with ring stack
					case state12Drive:
						robot.strafe(6, 0, 90, 1, 0, 0);
						
						if (robot.isStrafeComplete) {
							robot.setPower(0, 0, 0, 1);
							newState(MainState.state13Drive);
							break;
						}
						break;
					
					
					//quickly push over stack and separate the rings
					case state13Drive:
						robot.strafe(24, 0, 0, .5, .2, .3);
						intake.intakeStallControl();
						if (mainTime.seconds() > .7 && robot.isStrafeComplete) {
							shooter.shooterPID.resetIntegralSum();
							robot.setPower(0, 0, 0, 1);
							newState(MainState.state14Drive);
							break;
						}
						break;
					
					
					//slowly advance forward with intake to get 3 rings
					case state14Drive:
						robot.strafe(8, 0, 0, .3, .3, 0);
						intake.intakeStallControl();
						if (robot.isStrafeComplete) newState(MainState.state15Shoot);
						break;
					
					
					//shoot first three rings
					case state15Shoot:
						robot.setPowerAuto(0, 0, 0);
						shooter.setRPM(3100);
						
						if (mainTime.seconds() > .8) {
							intake.intakeOff();
							intake.retractBumper();
						}
						
						if (mainTime.seconds() > 1) shooter.feederState(true);
						if (mainTime.seconds() > 1.6 && Shooter.feederCount() > 4) newState(MainState.state18Turn);
						break;
					
					//turn towards target B
					case state18Turn:
						robot.turn(180, 1, 0);
						shooter.shooterOff();
						wobble.armPosition(.3);
						
						if (mainTime.seconds() > .5 && robot.isTurnComplete) {
							newState(MainState.state19Drive);
							robot.setPower(0, 0, 0, 1);
							break;
						}
						break;
					
					
					//drive to target B and open the gripper
					case state19Drive:
						robot.strafe(14, 180, 0, .6, .3, 0);
						
						if (robot.isStrafeComplete) {
							robot.setPower(0, 0, 0, 1);
							newState(MainState.state20WobbleGoal);
						}
						break;
					
					case state20WobbleGoal:
						wobble.armDown();
						intake.setBumperThreshold(1);
						robot.setPowerAuto(0,0,robot.closestTarget(180), 1);
						if(mainTime.seconds() > .8){
						
						}
						break;
					
				}
				
				break;
			
			case 4:
				switch (currentMainState) {
					//drive to target C
					case state1Drive:
						robot.strafe(75, 180, 0, 1, .3, 0);
						
						if (robot.currentInches > 60) wobble.armDown();
						
						if (robot.isStrafeComplete) {
							robot.setPower(0, 0, 0, 1);
							newState(MainState.state3Turn);
						}
						break;
					
					
					//turn to power shot shooting position
					case state3Turn:
						if (mainTime.seconds() > .8) robot.turn(-137, 1, .7);
						if (mainTime.seconds() > .9 && robot.isTurnComplete) newState(MainState.state4Drive);
						break;
					
					
					//drive to power shot shooting position
					case state4Drive:
						wobble.gripperOpen();
						
						if (mainTime.seconds() > 1){
							wobble.armFold();
							wobble.gripperHalf();
						}
						robot.strafe(40, -137, -137, 1, .3, 0);
						
						if (robot.isStrafeComplete) newState(MainState.state5Turn);
						break;
					
					
					//turn towards power shots
					case state5Turn:
						if (mainTime.seconds() > .8) {
							robot.turn(-2, 1, 0);
							shooter.powerShot();
						}
						if (mainTime.seconds() > .9 && robot.isTurnComplete) {
							newState(MainState.state6PS1);
							robot.setPowerAuto(0, 0, 0);
						}
						break;
					
					
					//shoot power shot 1, turn to power shot 2
					case state6PS1:
						shooter.powerShot();
						katana.katanaShoot();
						
						if (Shooter.feederCount() < 1) {
							robot.setPowerAuto(0, 0, 0);
							if (mainTime.seconds() > .5) shooter.feederState(true);
							else shooter.feederState(false);
						} else {
							robot.turn(8, 1, 1);
							shooter.feederState(false);
							if (mainTime.seconds() > .5 && robot.isTurnComplete) newState(MainState.state7PS2);
						}
						break;
					
					
					//shoot power shot 2, turn to power shot 3
					case state7PS2:
						shooter.powerShot();
						if (Shooter.feederCount() < 2) {
							robot.setPowerAuto(0, 0, 8);
							shooter.feederState(true);
						} else {
							robot.turn(11, 1, 1);
							shooter.feederState(false);
							if (robot.isTurnComplete) newState(MainState.state8PS3);
						}
						break;
					
					
					//shoot power shot 3, turn to second wobble goal
					case state8PS3:
						shooter.powerShot();
						wobble.armDown();
						wobble.gripperOpen();
						
						if (Shooter.feederCount() < 3) {
							shooter.feederState(true);
							robot.setPowerAuto(0, 0, 11);
						} else {
							robot.turn(-35, 1, .8);
							shooter.feederState(false);
							shooter.shooterOff();
							if (robot.isTurnComplete) newState(MainState.state9Drive);
						}
						break;
					
					
					//drive to second wobble goal
					case state9Drive:
						robot.strafe(19, -35, 145, 1, 0, 0);
						katana.katanaHalfFold();
						Shooter.setFeederCount(0);
						if (robot.isStrafeComplete) {
							newState(MainState.state10WobbleGoal);
							robot.setPower(0, 0, 0, 1);
						}
						break;
					
						
						
					case state10WobbleGoal:
						wobble.armDown();
						if (mainTime.seconds() > 1) wobble.gripperGrip();
						if (mainTime.seconds() > 1.6) {
							wobble.armTele();
							robot.strafe(3, -37, 143, 1, .3, 0);
							if (robot.isStrafeComplete) newState(MainState.state11Turn);
						}
						break;
					
						
					
					case state11Turn:
						robot.turn(0, 1, 1);
						intake.setBumperThreshold(3);
						if (robot.isTurnComplete) {
							newState(MainState.state12Drive);
							robot.setPower(0, 0, 0, 1);
						}
						break;
						
					
					//drive to target A
					case state12Drive:
						intake.setBumperThreshold(3);
						robot.strafe(15, 0, 90, .7, .2, 0);
						wobble.armUp();
						wobble.gripperGrip();
						if (robot.isStrafeComplete) newState(MainState.state13Drive);
						break;
					
					
					
					case state13Drive:
						wobble.armUp();
						wobble.gripperGrip();
						intake.setBumperThreshold(3);
						if(mainTime.seconds() > 0)robot.strafe(6,0,-90, .7,.1,0);
						if(mainTime.seconds() > 0 && robot.isStrafeComplete) newState(MainState.state14Drive);
						break;
					
					
					//strafe onto launch line
					case state14Drive:
						wobble.armUp();
						wobble.gripperGrip();
						intake.intakeOff();
						robot.strafe(3,0,180, .6, .3, 0);
						if(robot.isStrafeComplete) newState(MainState.state15Drive);
						break;
					
						
					case state15Drive:
						intake.setBumperThreshold(1);
						intake.intakeStallControl();
						if (mainTime.seconds() > .3) {
							robot.strafe(35, 0, 0, .25, 0, 0);
							if(robot.isStrafeComplete) newState(MainState.state16Shoot);
						}
						break;
					
						
					case state16Shoot:
						robot.setPowerAuto(0, 0, 0);
						shooter.highTower();
						katana.katanaShoot();
						
						if (mainTime.seconds() > 1.2) {
							intake.intakeOff();
							intake.retractBumper();
						}
						if (mainTime.seconds() > 1) shooter.feederState(true);
						if (mainTime.seconds() > 1.6 && Shooter.feederCount() >= 6) newState(MainState.state17Drive);
						break;
						
						
					case state17Drive:
						robot.strafe(36, 0, 0, 1, .2,0);
						shooter.shooterOff();
						katana.katanaHalfFold();
						if(robot.isStrafeComplete) newState(MainState.state18Turn);
						break;
						
						
					case state18Turn:
						if(mainTime.seconds() > .6){
							robot.turn(-105 , 1, .4);
							wobble.armPosition(.15);
						}
						if(mainTime.seconds() > .6 && robot.isTurnComplete) newState(MainState.state19Drive);
						break;
						
						
					case state19Drive:
						robot.strafe(7, -105, 75, .8, .2, 0);
						if(robot.isStrafeComplete){
							newState(MainState.state20Drive);
							wobble.armDown();
							wobble.gripperOpen();
						}
						break;
						
						
					case state20Drive:
						if(mainTime.seconds() > .2) robot.strafe(3, -105, -105, .5, .5, 0);
						if(mainTime.seconds() > .2 && robot.isStrafeComplete) newState(MainState.state21Drive);
						break;
					
						
					case state21Drive:
						robot.strafe(45, -90, 180, 1, .5, 0);
						wobble.armFold();
						wobble.gripperHalf();
						if(robot.isStrafeComplete) newState(MainState.stateFinished);
						break;
						
						
					case stateFinished:
						robot.setPowerAuto(0,0,-90);
				}
				
				break;
				
				
		}
		telemetry.update();
	}
	
	
	private void newState(MainState newState) {
		currentMainState = newState;
		mainTime.reset();
	}
	
	private enum MainState {
		state1Drive,
		state2WobbleGoal,
		state3Turn,
		state4Drive,
		state5Turn,
		state6PS1,
		state7PS2,
		state8PS3,
		state9Drive,
		state10WobbleGoal,
		state11Drive,
		state11Turn,
		state12WobbleGoal,
		state12Drive,
		state13Turn,
		state13Drive,
		state14Drive,
		state15Drive,
		state15Shoot,
		state16Shoot,
		state16Drive,
		state17Shoot,
		state17Drive,
		state18Turn,
		state18Drive,
		state18Shoot,
		state19Drive,
		state20Drive,
		state20WobbleGoal,
		state21Drive,
		stateFinished,
		state1Diagonal,
		state1Turn
	}
}
