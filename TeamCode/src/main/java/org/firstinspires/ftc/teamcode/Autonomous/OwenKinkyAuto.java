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

@Autonomous(name = "Owen Kinky Auto", group = "Auto")

public class OwenKinkyAuto extends OpMode {
	
	private final ElapsedTime feederTime = new ElapsedTime();
	private final ElapsedTime mainTime = new ElapsedTime();
	
	private Controller operator;
	private Gyro gyro;
	private MecanumChassis robot;
	private Shooter shooter;
	private Intake intake;
	private WobbleGripper wobble;
	boolean isFeederLocked = true;
	
	private MainState currentMainState = MainState.state1;
//	OpenCvCamera webcam;

	//private FtcDashboard dashboard = FtcDashboard.getInstance();
	//private Telemetry dashboardTelemetry = dashboard.getTelemetry();
	//private MultipleTelemetry multTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);


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
		gyro = new Gyro(imu, 0);
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
					case state1: //move forward to first wobble goal position
						robot.strafe(43, 180, 0, 1, 0, 0);
						
						if (robot.currentInches > 20){
							wobble.armDown();
						}
						
						if (robot.isStrafeComplete) {
							robot.setPower(0,0,0,1);
							newState(MainState.state1WobbleGoal);
						}
						telemetry.addData("isStrafeComplete = ", robot.isStrafeComplete);
						
						break;
						
					case state1WobbleGoal: //put down wobble goal
						wobble.armDown();
						if(mainTime.seconds() > .9){
							wobble.gripperOpen();
							robot.strafe(4, 180, 180, .5, 0, 0);
						}else{
							robot.setPower(0,0,0,1);
						}
						
						if(mainTime.seconds() > .9 && robot.isStrafeComplete) {
							newState(MainState.state1Turn);
							wobble.armFold();
							wobble.gripperHalf();
						}
						break;
						
					case state1Turn: //turn
						robot.turn(270, 1, 0);
						if (robot.isTurnComplete) {
							newState(MainState.state2);
						}
						telemetry.addData("current state = ", "turn");
						telemetry.addData("current state = ", "turn");
						break;
					
					case state2: //moves to waypoint 2 in front of second powershot behind launch line
						robot.strafe(28, 270, 270, 1, 0, 0);
						if (robot.isStrafeComplete) {
							newState(MainState.state2Turn);
						}
						break;
						
					case state2Turn: //turn towards center powershot
						if(mainTime.seconds() > .8){ robot.turn(0, 1, 0); }
						if (mainTime.seconds() > .8 && robot.isTurnComplete) {
							newState(MainState.statePS1);
						}
						break;
						
					case statePS1: //do powershot shooter code
						shooter.powerShot();
						
						if (shooter.feederCount() < 1) {
							robot.setPowerAuto(0,0,0,1);
							if(mainTime.seconds() > 1.5){ shooter.feederState(true); }
						}else{
							robot.turn(-5, 1, 1);
							if (robot.isTurnComplete) {
								newState(MainState.statePS2);
								break;
							}
							break;
						}
						break;
					
					case statePS2: //do powershot shooter code
						shooter.powerShot();
						if (shooter.feederCount() < 2) {
							robot.setPowerAuto(0,0,-6,1);
							if(mainTime.seconds() > 0){ shooter.feederState(true); }
						}else{
							robot.turn(-10, 1, 1);
							if (robot.isTurnComplete) {
								newState(MainState.statePS3);
								break;
							}
							break;
						}
						break;
						
					case statePS3: //do powershot shooter code
						shooter.powerShot();
						wobble.armDown();
						wobble.gripperOpen();
						if (shooter.feederCount() < 3) {
							robot.setPowerAuto(0,0,-11,1);
							if(mainTime.seconds() > 0){ shooter.feederState(true); }
						}else{
							robot.turn(-24, 1, .8);
							shooter.shooterOff();
							if (robot.isTurnComplete) {
								newState(MainState.state3);
								
								break;
							}
							break;
						}
						break;
						
					case state3: //move to waypoint3
						robot.strafe(19, -24, 157, 1, 0, 0);
						if (robot.isStrafeComplete) {
							newState(MainState.state3WobbleGoal);
							robot.setPower(0,0,0,1);
						}
						break;
					
					case state3WobbleGoal: //put down wobble goal
						wobble.armDown();
						if(mainTime.seconds() > .8) {
							wobble.gripperGrip();
						}
						if(mainTime.seconds() > 1.6) {
							robot.turn(215,  1, 0);
							if(robot.isTurnComplete) { newState(MainState.state4); }
						}
						break;
						
					case state4: //move forward to fourth wobble goal position
						robot.strafe(35, 213, 33, 1, 0, 0);
						if (robot.isStrafeComplete) {
							newState(MainState.state4WobbleGoal);
						}
						break;
					
					case state4WobbleGoal: //put down wobble goal
						wobble.armDown();
						if(mainTime.seconds() > .9){
							wobble.gripperOpen();
							robot.strafe(8, 215, 270, .5, 0, 0);
						}else{
							robot.setPower(0,0,0,1);
						}
						
						if(mainTime.seconds() > 1 && robot.isStrafeComplete) {
							newState(MainState.state5);
							wobble.armFold();
							wobble.gripperHalf();
						}
						break;
						
					case state5: //move forward to fourth wobble goal position
						robot.turn(-90, .9, .5);
						if (mainTime.seconds() > 1 && robot.isStrafeComplete) {
							newState(MainState.state6);
						}
						break;
						
					case state6:
						robot.strafe(10,-90,0, .5,0,0);
						if(robot.isStrafeComplete){ robot.setPower(0,0,0,1); newState(MainState.stateFinished); }
						break;

				}

				break;

			case 1:
				break;

			case 4:
				switch (currentMainState) {
					case state1: //move forward to first wobble goal position
						robot.strafe(82, 180, 0, 1, 0, 0);
						
						if (robot.currentInches > 50){
							wobble.armDown();
						}
						
						if (robot.isStrafeComplete) {
							robot.setPower(0,0,0,1);
							newState(MainState.state1WobbleGoal);
						}
						telemetry.addData("isStrafeComplete = ", robot.isStrafeComplete);
						
						break;
					
					case state1WobbleGoal: //put down wobble goal
						wobble.armDown();
						if(mainTime.seconds() > .9){
							wobble.gripperOpen();
							robot.strafe(4, 180, 180, .5, 0, 0);
						}else{
							robot.setPower(0,0,0,1);
						}
						
						if(mainTime.seconds() > .9 && robot.isStrafeComplete) {
							newState(MainState.state1Turn);
							wobble.armFold();
							wobble.gripperHalf();
						}
						break;
					
					case state1Turn: //turn
						robot.turn(-139, 1, 0);
						if (robot.isTurnComplete) {
							newState(MainState.state2);
						}
						break;
					
					case state2: //moves to waypoint 2 in front of second powershot behind launch line
						robot.strafe(44, -139, -139, 1, 0, 0);
						if (robot.isStrafeComplete) {
							newState(MainState.state2Turn);
						}
						break;
					
					case state2Turn: //turn towards center powershot
						if(mainTime.seconds() > .8){ robot.turn(0, 1, 0); }
						if (mainTime.seconds() > .8 && robot.isTurnComplete) {
							newState(MainState.statePS1);
							robot.setPower(0,0,0,1);
						}
						break;
					
					case statePS1: //do powershot shooter code
						shooter.powerShot();
						
						if (shooter.feederCount() < 1) {
							robot.setPowerAuto(0,0,0,1);
							if(mainTime.seconds() > 1.5){ shooter.feederState(true); }
						}else{
							robot.turn(-5, 1, 1);
							if (robot.isTurnComplete) {
								newState(MainState.statePS2);
								break;
							}
							break;
						}
						break;
					
					case statePS2: //do powershot shooter code
						shooter.powerShot();
						if (shooter.feederCount() < 2) {
							
							if(mainTime.seconds() > 0){ shooter.feederState(true); }
						}else{
							robot.turn(-10, 1, 1);
							if (robot.isTurnComplete) {
								newState(MainState.statePS3);
								break;
							}
							break;
						}
						break;
					
					case statePS3: //do powershot shooter code
						shooter.powerShot();
						wobble.armDown();
						wobble.gripperOpen();
						if (shooter.feederCount() < 3) {
							
							if(mainTime.seconds() > 0){ shooter.feederState(true); }
						}else{
							robot.turn(-26, 1, .8);
							shooter.shooterOff();
							if (robot.isTurnComplete) {
								newState(MainState.state3);
								
								break;
							}
							break;
						}
						break;
					
					case state3: //move to waypoint3
						robot.strafe(18, -26, 154, 1, 0, 0);
						if (robot.isStrafeComplete) {
							newState(MainState.state3WobbleGoal);
							robot.setPower(0,0,0,1);
						}
						break;
					
					case state3WobbleGoal: //put down wobble goal
						wobble.armDown();
						if(mainTime.seconds() > 1) {
							wobble.gripperGrip();
						}
						if(mainTime.seconds() > 1.6) {
							wobble.armTele();
							robot.strafe(7, -26, 154, 1, 0, 0);
							if(robot.isStrafeComplete) { newState(MainState.state3Turn); robot.setPower(0,0,0,1); }
						}
						break;
					
					case state3Turn:
						robot.turn(0, 1, 1);
						intake.deployReach();
						if (robot.isTurnComplete) {
							newState(MainState.state4);
							robot.setPower(0,0,0,1);
						}
						break;
						
					case state4:
						robot.strafe(11,0,90,.75,0,0);
						if(robot.isStrafeComplete) {
							robot.setPower(0,0,0,1);
							newState(MainState.state5);
							break;
						}
						break;
						
					case state5:
						if(robot.currentInches > 25){
							intake.intakeOn();
						}
						robot.strafe(75, 0, 0, .3, .2 ,.2);
						
						if(mainTime.seconds() > .7 && robot.isStrafeComplete){
							robot.setPower(0,0,0,1);
							newState(MainState.state5Shoot);
							break;
						}
						break;
						
					case state5Shoot:
						robot.setPowerAuto(0,0,0,1);
						shooter.topGoal();
						if(mainTime.seconds() > 1) {
							intake.intakeOff();
							shooter.feederState(true);
						}
						if(mainTime.seconds() > 1.5 && shooter.feederCount() >=6){
							newState(MainState.state6);
							break;
						}
						break;
						
					case state6:
						shooter.topGoal();
						intake.intakeOn();
						if(mainTime.seconds() > 2){ intake.intakeOff(); newState(MainState.state6Shoot); }
						break;
						
					case state6Shoot:
						robot.setPowerAuto(0,0,0,1);
						shooter.topGoal();
						shooter.feederState(true);
						if(shooter.feederCount() >=9){ newState(MainState.state7Turn); }
						break;
						
					case state7Turn:
						robot.turn(195,1,0);
						shooter.shooterOff();
						wobble.armPostion(.14);
						if(mainTime.seconds() > .5 && robot.isTurnComplete) {
							newState(MainState.state7);
							robot.setPower(0,0,0,1);
							break;
						}
						break;
						
					case state7:
						robot.strafe(45,195,15, 1, .5, 0);
						
						if(robot.isStrafeComplete) {
							robot.setPower(0,0,0,1);
							newState(MainState.stateFinished);
						}
						break;
				}
				
				break;
		}
		telemetry.addData("flywheel rpm = ", Math.abs(shooter.updateRPM()));
		telemetry.update();
		
		
		
		
	}
	

	
	
	private void newState(MainState newState) {
		currentMainState = newState;
		mainTime.reset();
	}
	
	private enum MainState {
		state1,
		state1Turn,
		state1WobbleGoal,
		state2,
		state2Turn,
		statePS1,
		statePS2,
		statePS3,
		state3,
		state3Turn,
		state3WobbleGoal,
		state4,
		state4Turn,
		state4Strafe,
		state4WobbleGoal,
		state5,
		state5Shoot,
		state6,
		state6Shoot,
		state7Turn,
		state7,
		stateFinished

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
