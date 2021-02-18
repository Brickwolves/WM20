package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Gyro;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumChassis;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.WobbleGripper;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "Owen Final Auto", group = "Auto")

public class OwenFinalAuto extends OpMode {
	
	private Controller operator;
	private MecanumChassis robot;
	private Shooter shooter;
	private Intake intake;
	private WobbleGripper wobble;
	OpenCvCamera webcam;
	
	private MainState currentMainState = MainState.state1Drive;
	private final ElapsedTime mainTime = new ElapsedTime();
	

	private static double ringCount = 0;
	private boolean ringsFound = false;

	@Override
	public void init() {
		Servo feeder = hardwareMap.get(Servo.class, "feeder");
		Servo feederLock = hardwareMap.get(Servo.class, "feederlock");
		
		Servo outerRollerOne = hardwareMap.get(Servo.class, "outerrollerone");
		Servo outerRollerTwo = hardwareMap.get(Servo.class, "outerrollertwo");
		
		Servo lifter = hardwareMap.get(Servo.class, "lifter");
		Servo gripperOne = hardwareMap.get(Servo.class, "gripperone");
		Servo gripperTwo = hardwareMap.get(Servo.class, "grippertwo");
		
		DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
		DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontright");
		DcMotor backLeft = hardwareMap.get(DcMotor.class, "backleft");
		DcMotor backRight = hardwareMap.get(DcMotor.class, "backright");
		
		DcMotor shooterOne = hardwareMap.get(DcMotor.class, "shooterone");
		DcMotor shooterTwo = hardwareMap.get(DcMotor.class, "shootertwo");
		DcMotor intakeDrive = hardwareMap.get(DcMotor.class, "intake");
		
		operator = new Controller(gamepad2);
		
		Utils.setHardwareMap(hardwareMap);
		IMU imu = new IMU("imu");
		Gyro gyro = new Gyro(imu, 0);
		shooter = new Shooter(shooterOne, shooterTwo, feeder, feederLock);
		intake = new Intake(intakeDrive, outerRollerOne, outerRollerTwo);
		robot = new MecanumChassis(frontLeft, frontRight, backLeft, backRight, gyro);
		wobble = new WobbleGripper(gripperOne, gripperTwo, lifter);
	}
	
	public void init_loop(){
		shooter.resetFeeder();
		shooter.lockFeeder();
		if(operator.RBToggle()){
			wobble.gripperGrip();
		}else{
			wobble.gripperHalf();
		}
		if(operator.crossToggle()){
			intake.intakeOn();
			intake.deployReach();
		}else{
			intake.intakeOff();
			intake.retractReach();
		}
		
//
//		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//		webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
//
//		webcam.setPipeline(new OwenKinkyAuto.RingDetectingPipeline());
//		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//		{
//			@Override
//			public void onOpened()
//			{
//				webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
//			}
//		});
	}
	
	public void start() {
		mainTime.reset();
		robot.resetGyro(180);
		robot.resetMotors();
		ringCount = 4;
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void loop() {
		
		telemetry.addData("strafe drive = ", robot.strafeDrive);
		telemetry.addData("auto drive = ", robot.autoDrive);
		telemetry.addData("current state: ", currentMainState);
		
		switch ((int) ringCount) {
			case 0:
				switch (currentMainState) {
					//drive to target A
					case state1Drive:
						robot.strafe(40, 180, 0, 1, 0, 0);
						
						if (robot.currentInches > 20){
							wobble.armDown();
						}
						
						if (robot.isStrafeComplete) {
							robot.setPower(0,0,0,1);
							newState(MainState.state2WobbleGoal);
						}
						
						break;
					
					//put down wobble goal and back up
					case state2WobbleGoal:
						wobble.armDown();
						if(mainTime.seconds() > .9){
							wobble.gripperOpen();
							robot.strafe(4, 180, 180, .5, 0, 0);
						}else{
							robot.setPower(0,0,0,1);
						}
						
						if(mainTime.seconds() > .9 && robot.isStrafeComplete) {
							newState(MainState.state3Turn);
							wobble.armFold();
							wobble.gripperHalf();
						}
						break;
					
						
					//turn towards power shot shooting position
					case state3Turn: //turn
						robot.turn(270, 1, 0);
						
						if (robot.isTurnComplete) { newState(MainState.state4Drive); }
						break;
						
						
					//drive to power shot shooting position
					case state4Drive:
						robot.strafe(25, 270, 270, 1, 0, 0);
						
						if (robot.isStrafeComplete) { newState(MainState.state5Turn); }
						break;
						
						
					//turn to face powershots
					case state5Turn:
						if(mainTime.seconds() > .8){ robot.turn(0, 1, 0); }
						
						if (mainTime.seconds() > .8 && robot.isTurnComplete) { newState(MainState.state6PS1); }
						break;
						
						
					//shoot power shot 1, turn to power shot 2
					case state6PS1:
						shooter.powerShot();
						
						if (shooter.feederCount() < 1) {
							robot.setPowerAuto(0,0,0,1);
							
							if(mainTime.seconds() > 1.5){ shooter.feederState(true); }
						}else{
							robot.turn(-5, 1, 1);
							
							if (robot.isTurnComplete) { newState(MainState.state7PS2); }
						}
						break;
					
						
					//shoot power shot 2, turn to power shot 3
					case state7PS2:
						shooter.powerShot();
						if (shooter.feederCount() < 2) {
							robot.setPowerAuto(0,0,-6,1);
							
							if(mainTime.seconds() > 0){ shooter.feederState(true); }
						}else{
							robot.turn(-10, 1, 1);
							
							if (robot.isTurnComplete) { newState(MainState.state8PS3); }
						}
						break;
					
						
					//shoot power shot 3, turn to second wobble goal
					case state8PS3:
						shooter.powerShot();
						wobble.armDown();
						wobble.gripperOpen();
						
						if (shooter.feederCount() < 3) {
							robot.setPowerAuto(0,0,-11,1);
							if(mainTime.seconds() > 0){ shooter.feederState(true); }
						}else{
							robot.turn(-24, 1, .8);
							shooter.shooterOff();
							
							if (robot.isTurnComplete) { newState(MainState.state9Drive); }
						}
						break;
						
						
					//drive to second wobble goal
					case state9Drive:
						robot.strafe(17, -24, 157, 1, 0, 0);
						if (robot.isStrafeComplete) {
							newState(MainState.state10WobbleGoal);
							robot.setPower(0,0,0,1);
						}
						break;
						
						
					//pick up second wobble goal, turn away from target A
					case state10WobbleGoal:
						wobble.armDown();
						
						if(mainTime.seconds() > .8) { wobble.gripperGrip(); }
						
						if(mainTime.seconds() > 1.6) {
							robot.turn(215,  1, 0);
							if(robot.isTurnComplete) { newState(MainState.state11Drive); }
						}
						break;
						
						
					//drive to targeet A
					case state11Drive:
						robot.strafe(35, 213, 33, 1, 0, 0);
						
						if (robot.isStrafeComplete) { newState(MainState.state12WobbleGoal); }
						break;
					
						
					//put down second wobble goal and back up
					case state12WobbleGoal:
						wobble.armDown();
						if(mainTime.seconds() > .9){
							wobble.gripperOpen();
							robot.strafe(8, 215, 270, .5, 0, 0);
						}else{
							robot.setPower(0,0,0,1);
						}
						
						if(mainTime.seconds() > 1 && robot.isStrafeComplete) {
							newState(MainState.state13Turn);
							wobble.armFold();
							wobble.gripperHalf();
						}
						break;
						
						
					//turn to set teleop angle to 0
					case state13Turn:
						robot.turn(-90, .9, .5);
						if (mainTime.seconds() > 1 && robot.isStrafeComplete) { newState(MainState.state14Drive); }
						break;
						
						
					//strafe onto launch line
					case state14Drive:
						robot.strafe(10,-90,0, .5,0,0);
						
						if(robot.isStrafeComplete){ robot.setPower(0,0,0,1); newState(MainState.stateFinished); }
						break;
				}

				break;

			case 1:
				switch (currentMainState) {
					//drive to WP1
					case state1Drive:
						robot.strafe(80, 180, 0, 1, 0, 0);

						if (robot.currentInches > 50){ wobble.armDown(); }

						if (robot.isStrafeComplete) {
							robot.setPower(0,0,0,1);
							newState(MainState.state1Turn);
						}
						break;
					case state1Turn:
						robot.turn(-90, .9, .5);
						if (mainTime.seconds() > 1 && robot.isStrafeComplete) { newState(MainState.state1Diagonal); }
						break;

					case state1Diagonal:
						robot.strafe(80, 180, 0, 1, 0, 0);

						if (robot.currentInches > 50){ wobble.armDown(); }

						if (robot.isStrafeComplete) {
							robot.setPower(0,0,0,1);
							newState(MainState.state2WobbleGoal);
						}
						break;

					//put down wobble goal and back up
					case state2WobbleGoal:
						wobble.armDown();
						if(mainTime.seconds() > .9){
							wobble.gripperOpen();
							robot.strafe(4, 180, 180, .5, 0, 0);
						}else{
							robot.setPower(0,0,0,1);
						}
						if(mainTime.seconds() > .9 && robot.isStrafeComplete) {
							newState(MainState.state3Turn);
							wobble.armFold();
							wobble.gripperHalf();
						}
						break;


					//turn to power shot shooting position
					case state3Turn:
						robot.turn(-140, 1, 0);

						if (robot.isTurnComplete) { newState(MainState.state4Drive); }
						break;


					//drive to power shot shooting position
					case state4Drive:
						robot.strafe(41, -140, -139, 1, 0, 0);

						if (robot.isStrafeComplete) { newState(MainState.state5Turn); }
						break;


					//turn towards power shots
					case state5Turn:
						if(mainTime.seconds() > .8){ robot.turn(0, 1, 0); }

						if (mainTime.seconds() > .8 && robot.isTurnComplete) {
							newState(MainState.state6PS1);
						}
						break;


					//shoot power shot 1, turn to power shot 2
					case state6PS1:
						shooter.powerShot();

						if (shooter.feederCount() < 1) {
							robot.setPowerAuto(0,0,0);
							if(mainTime.seconds() > 1.5){ shooter.feederState(true); }
						}else{
							robot.turn(-4.5, 1, 1);

							if (robot.isTurnComplete) { newState(MainState.state7PS2); }
						}
						break;


					//shoot power shot 2, turn to power shot 3
					case state7PS2:
						shooter.powerShot();
						if (shooter.feederCount() < 2) {
							robot.setPowerAuto(0,0,-4.5);
							if(mainTime.seconds() > 0){ shooter.feederState(true); }
						}else{
							robot.turn(-4.5, 1, 1);

							if (robot.isTurnComplete) { newState(MainState.state8PS3); }
						}
						break;


					//shoot power shot 3, turn to second wobble goal
					case state8PS3:
						shooter.powerShot();
						wobble.armDown();
						wobble.gripperOpen();

						if (shooter.feederCount() < 3) {
							if(mainTime.seconds() > 0){ shooter.feederState(true); }
							robot.setPowerAuto(0,0,-10);
						}else{
							robot.turn(-28, 1, .8);
							shooter.shooterOff();

							if (robot.isTurnComplete) { newState(MainState.state9Drive); }
						}
						break;


					//drive to second wobblee goal
					case state9Drive:
						robot.strafe(16, -28, 154, 1, 0, 0);
						if (robot.isStrafeComplete) {
							newState(MainState.state10WobbleGoal);
							robot.setPower(0,0,0,1);
						}
						break;


					//pick up second wobble goal
					case state10WobbleGoal:
						wobble.armDown();

						if(mainTime.seconds() > 1) { wobble.gripperGrip(); }

						if(mainTime.seconds() > 1.6) {
							wobble.armTele();
							robot.strafe(7, -26, 154, 1, 0, 0);

							if(robot.isStrafeComplete) { newState(MainState.state11Turn); robot.setPower(0,0,0,1); }
						}
						break;


					//turn to face back wall
					case state11Turn:
						robot.turn(0, 1, 1);
						intake.deployReach();

						if (robot.isTurnComplete) {
							newState(MainState.state12Drive);
							robot.setPower(0,0,0,1);
						}
						break;


					//strafe to line up with ring stack
					case state12Drive:
						robot.strafe(11,0,90,1,0,0);

						if(robot.isStrafeComplete) {
							robot.setPower(0,0,0,1);
							newState(MainState.state13Drive);
							break;
						}
						break;


					//quickly push over stack and separate the rings
					case state13Drive:
						robot.strafe(22, 0, 0, .6, .2 ,.3);

						if(mainTime.seconds() > .7 && robot.isStrafeComplete){
							shooter.shooterPID.resetIntegralSum();
							robot.setPower(0,0,0,1);
							newState(MainState.state14Drive);
							break;
						}
						break;


					//slowly advance forward with intake to get 3 rings
					case state14Drive:
						robot.strafe(20,0,0,.3,.3,0);
						shooter.setRPM(3100);
						intake.intakeOn();

						if(robot.isStrafeComplete){ newState(MainState.state15Shoot); }
						break;


					//shoot first three rings
					case state15Shoot:
						robot.setPowerAuto(0, 0, 0);
						shooter.setRPM(3100);

						if(mainTime.seconds()>1.5) {
							shooter.feederState(true);
							intake.intakeOff();
						}
						if(mainTime.seconds() > 1.6 && shooter.feederCount() >=6){ newState(MainState.state18Turn); }
						break;

					//turn towards target C
					case state18Turn:
						robot.turn(199,1,0);
						shooter.shooterOff();
						wobble.armPostion(.14);

						if(mainTime.seconds() > .5 && robot.isTurnComplete) {
							newState(MainState.state19Drive);
							robot.setPower(0,0,0,1);
							break;
						}
						break;


					//drive to target C and open the gripper
					case state19Drive:
						robot.strafe(55,199,11, 1, .5, 0);

						if(robot.isStrafeComplete) {
							robot.setPower(0,0,0,1);
							wobble.gripperOpen();
							newState(MainState.stateFinished);
						}
						break;


					//turn to set teleop angle to 0
					case stateFinished:
						robot.turn(-90,1,0);
				}

				break;

			case 4:
				switch (currentMainState) {
					//drive to target C
					case state1Drive:
						robot.strafe(80, 180, 0, 1, 0, 0);
						
						if (robot.currentInches > 50){ wobble.armDown(); }
						
						if (robot.isStrafeComplete) {
							robot.setPower(0,0,0,1);
							newState(MainState.state2WobbleGoal);
						}
						break;
					
						
					//put down wobble goal and back up
					case state2WobbleGoal:
						wobble.armDown();
						if(mainTime.seconds() > .9){
							wobble.gripperOpen();
							robot.strafe(4, 180, 180, .5, 0, 0);
						}else{
							robot.setPower(0,0,0,1);
						}
						if(mainTime.seconds() > .9 && robot.isStrafeComplete) {
							newState(MainState.state3Turn);
							wobble.armFold();
							wobble.gripperHalf();
						}
						break;
						
						
					//turn to power shot shooting position
					case state3Turn:
						robot.turn(-140, 1, 0);
						
						if (robot.isTurnComplete) { newState(MainState.state4Drive); }
						break;
						
						
					//drive to power shot shooting position
					case state4Drive:
						robot.strafe(41, -140, -139, 1, 0, 0);
						
						if (robot.isStrafeComplete) { newState(MainState.state5Turn); }
						break;
					
						
					//turn towards power shots
					case state5Turn:
						if(mainTime.seconds() > .8){ robot.turn(0, 1, 0); }
						
						if (mainTime.seconds() > .8 && robot.isTurnComplete) {
							newState(MainState.state6PS1);
						}
						break;
					
					
					//shoot power shot 1, turn to power shot 2
					case state6PS1:
						shooter.powerShot();
						
						if (shooter.feederCount() < 1) {
							robot.setPowerAuto(0,0,0);
							if(mainTime.seconds() > 1.5){ shooter.feederState(true); }
						}else{
							robot.turn(-4.5, 1, 1);
							
							if (robot.isTurnComplete) { newState(MainState.state7PS2); }
						}
						break;
					
					
					//shoot power shot 2, turn to power shot 3
					case state7PS2:
						shooter.powerShot();
						if (shooter.feederCount() < 2) {
							robot.setPowerAuto(0,0,-4.5);
							if(mainTime.seconds() > 0){ shooter.feederState(true); }
						}else{
							robot.turn(-4.5, 1, 1);
							
							if (robot.isTurnComplete) { newState(MainState.state8PS3); }
						}
						break;
					
					
					//shoot power shot 3, turn to second wobble goal
					case state8PS3:
						shooter.powerShot();
						wobble.armDown();
						wobble.gripperOpen();
						
						if (shooter.feederCount() < 3) {
							if(mainTime.seconds() > 0){ shooter.feederState(true); }
							robot.setPowerAuto(0,0,-10);
						}else{
							robot.turn(-28, 1, .8);
							shooter.shooterOff();
							
							if (robot.isTurnComplete) { newState(MainState.state9Drive); }
						}
						break;
						
						
					//drive to second wobblee goal
					case state9Drive:
						robot.strafe(16, -28, 154, 1, 0, 0);
						if (robot.isStrafeComplete) {
							newState(MainState.state10WobbleGoal);
							robot.setPower(0,0,0,1);
						}
						break;
					
						
					//pick up second wobble goal
					case state10WobbleGoal:
						wobble.armDown();
						
						if(mainTime.seconds() > 1) { wobble.gripperGrip(); }
						
						if(mainTime.seconds() > 1.6) {
							wobble.armTele();
							robot.strafe(7, -26, 154, 1, 0, 0);
							
							if(robot.isStrafeComplete) { newState(MainState.state11Turn); robot.setPower(0,0,0,1); }
						}
						break;
					
						
					//turn to face back wall
					case state11Turn:
						robot.turn(0, 1, 1);
						intake.deployReach();
						
						if (robot.isTurnComplete) {
							newState(MainState.state12Drive);
							robot.setPower(0,0,0,1);
						}
						break;
						
						
					//strafe to line up with ring stack
					case state12Drive:
						robot.strafe(11,0,90,1,0,0);
						
						if(robot.isStrafeComplete) {
							robot.setPower(0,0,0,1);
							newState(MainState.state13Drive);
							break;
						}
						break;
						
						
					//quickly push over stack and separate the rings
					case state13Drive:
						robot.strafe(22, 0, 0, .6, .2 ,.3);
						
						if(mainTime.seconds() > .7 && robot.isStrafeComplete){
							shooter.shooterPID.resetIntegralSum();
							robot.setPower(0,0,0,1);
							newState(MainState.state14Drive);
							break;
						}
						break;
						
						
					//slowly advance forward with intake to get 3 rings
					case state14Drive:
						robot.strafe(20,0,0,.3,.3,0);
						shooter.setRPM(3100);
						intake.intakeOn();
						
						if(robot.isStrafeComplete){ newState(MainState.state15Shoot); }
						break;
						
						
					//shoot first three rings
					case state15Shoot:
						robot.setPowerAuto(0, 0, 0);
						shooter.setRPM(3100);
						
						if(mainTime.seconds()>1.5) {
							shooter.feederState(true);
							intake.intakeOff();
						}
						if(mainTime.seconds() > 1.6 && shooter.feederCount() >=6){ newState(MainState.state16Drive); }
						break;
						
						
					//intake last ring
					case state16Drive:
						shooter.topGoal();
						intake.intakeOn();
						robot.strafe(7,0,0,1,0,0);
						
						if(robot.isStrafeComplete){  newState(MainState.state17Shoot); }
						break;
						
						
					//shoot last ring
					case state17Shoot:
						robot.setPowerAuto(0,0,0);
						intake.retractReach();
						shooter.topGoal();
						shooter.feederState(true);
						
						if(shooter.feederCount() >=9){ newState(MainState.state18Turn); }
						break;
						
						
					//turn towards target C
					case state18Turn:
						robot.turn(199,1,0);
						shooter.shooterOff();
						wobble.armPostion(.14);
						
						if(mainTime.seconds() > .5 && robot.isTurnComplete) {
							newState(MainState.state19Drive);
							robot.setPower(0,0,0,1);
							break;
						}
						break;
						
						
					//drive to target C and open the gripper
					case state19Drive:
						robot.strafe(55,199,11, 1, .5, 0);
						
						if(robot.isStrafeComplete) {
							robot.setPower(0,0,0,1);
							wobble.gripperOpen();
							newState(MainState.stateFinished);
						}
						break;
						
					
					//turn to set teleop angle to 0
					case stateFinished:
						robot.turn(-90,1,0);
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
		state15Shoot,
		state16Drive,
		state17Shoot,
		state18Turn,
		state19Drive,
		stateFinished,
		state1Diagonal,
		state1Turn
	}

//	class RingDetectingPipeline extends OpenCvPipeline
//	{
//		boolean viewportPaused;
//
//		// Init mats here so we don't repeat
//		Mat YCbCr = new Mat();
//		Mat outPut = new Mat();
//		Mat upperCrop = new Mat();
//		Mat lowerCrop = new Mat();
//
//		// Rectangles starting coordinates      // Rectangles starting percentages
//		int rectTopX1; int rectTopX2;           //double rectTopX1Percent = 0; double rectTopX2Percent = 0;
//		int rectTopY1; int rectTopY2;           //double rectTopY1Percent = 0; double rectTopY2Percent = 0;
//
//		// Rectangles starting coordinates      // Rectangles starting percentages
//		int rectBottomX1; int rectBottomX2;     //double rectBottomX1Percent = 0; double rectBottomX2Percent = 0;
//		int rectBottomY1; int rectBottomY2;     //double rectBottomY1Percent = 0; double rectBottomY2Percent = 0;
//
//
//		@Override
//		public Mat processFrame(Mat input)
//		{
//			// Convert & Copy to outPut image
//			Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
//			input.copyTo(outPut);
//
//			// Dimensions for top rectangle
//			rectTopX1 = (int) (input.rows() * DashVision.rectTopX1Percent);
//			rectTopX2 = (int) (input.rows() * DashVision.rectTopX2Percent) - rectTopX1;
//			rectTopY1 = (int) (input.cols() * DashVision.rectTopY1Percent);
//			rectTopY2 = (int) (input.cols() * DashVision.rectTopY2Percent) - rectTopY1;
//
//			// Dimensions for bottom rectangle
//			rectBottomX1 = (int) (input.rows() * DashVision.rectBottomX1Percent);
//			rectBottomX2 = (int) (input.rows() * DashVision.rectBottomX2Percent) - rectBottomX1;
//			rectBottomY1 = (int) (input.cols() * DashVision.rectBottomY1Percent);
//			rectBottomY2 = (int) (input.cols() * DashVision.rectBottomY2Percent) - rectBottomY1;
//
//			// VISUALIZATION: Create rectangles and scalars, then draw them onto outPut
//			Scalar rectangleColor = new Scalar(0, 0, 255);
//			Rect rectTop = new Rect(rectTopX1, rectTopY1, rectTopX2, rectTopY2);
//			Rect rectBottom = new Rect(rectBottomX1, rectBottomY1, rectBottomX2, rectBottomY2);
//			Imgproc.rectangle(outPut, rectTop, rectangleColor, 2);
//			Imgproc.rectangle(outPut, rectBottom, rectangleColor, 2);
//
//
//
//
//			// IDENTIFY RINGS //
//
//			// Crop
//			upperCrop = YCbCr.submat(rectTop);
//			lowerCrop = YCbCr.submat(rectBottom);
//
//			// Extract Channels [Y, Cr, Cb], where 2 = index of Cb channel
//			Core.extractChannel(lowerCrop, lowerCrop, 2);
//			Core.extractChannel(upperCrop, upperCrop, 2);
//
//			// Store Averages
//			Scalar lowerAveOrange = Core.mean(lowerCrop);
//			Scalar upperAveOrange = Core.mean(upperCrop);
//			double finalLowerAve = lowerAveOrange.val[0];
//			double finalUpperAve = upperAveOrange.val[0];
//
//
//			// Check 4 rings
//			if (
//
//					finalUpperAve > DashVision.orangeMin &&
//							finalUpperAve < DashVision.orangeMax
//
//			) ringCount = 4.0;
//				// Check 0 rings
//			else if (
//
//					finalLowerAve > DashVision.orangeMax ||
//							finalLowerAve < DashVision.orangeMin
//			) ringCount = 0.0;
//			else ringCount = 1.0;
//
//			/**
//			 * RECT_BOTTOM_X1: 0.75
//			 * RECT_BOTTOM_X2: 0.9
//			 * RECT_BOTTOM_Y1: 0.38
//			 * RECT_BOTTOM_Y2: 0.42
//			 * RECT_TOP_X1: 0.75
//			 * RECT_TOP_X2: 0.9
//			 * RECT_TOP_Y1: 0.3
//			 * RECT_TOP_Y2: 0.38
//			 * Given a distance of around 3ft from rings
//			 */
//
//
//			multTelemetry.addData("Ring Count", ringCount);
//			multTelemetry.addData("finalLowerAve: ", finalLowerAve);
//			multTelemetry.addData("finalUpperAve: ", finalUpperAve);
//			multTelemetry.update();
//
//			// Return altered image
//			return outPut;
//		}
//
//		@Override
//		public void onViewportTapped()
//		{
//			viewportPaused = !viewportPaused;
//
//			if(viewportPaused)
//			{
//				webcam.pauseViewport();
//			}
//			else
//			{
//				webcam.resumeViewport();
//			}
//		}
//	}
}
