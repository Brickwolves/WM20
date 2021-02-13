package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.AutoUtils.DashConstants;
import org.firstinspires.ftc.teamcode.Autonomous.AutoUtils.DashVision;
import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Gyro;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumChassis;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.WobbleGripper;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

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
		intake.retractReach();
		intake.intakeOff();
		shooter.resetFeeder();
		shooter.lockFeeder();
		if(operator.RBToggle()){
			wobble.grip();
		}else{
			wobble.half();
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
		ringCount = 0;
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void loop() {
		

		telemetry.addData("strafe drive = ", robot.strafeDrive);
		telemetry.addData("auto drive = ", robot.autoDrive);
		switch ((int) ringCount) {
			case 0:
				switch (currentMainState) {
					case state1: //move forward to first wobble goal position
						robot.strafe(org.firstinspires.ftc.teamcode.Autonomous.AutoUtils.Utils.convertInches2Ticks(63), 180, 0, 1, 0, 0);
						
						if (robot.isStrafeComplete) {
							robot.setPower(0,0,0,0);
							newState(MainState.state1Turn);
							break;
						}
						telemetry.addData("isStrafeComplete = ", robot.isStrafeComplete);
						
						break;
					case state1Turn: //turn
						robot.turn(255, .8, 1);
						if (robot.isTurnComplete) {
							newState(MainState.state1WobbleGoal);
						}
						telemetry.addData("current state = ", "turn");
						telemetry.addData("current state = ", "turn");
						break;
					case state1WobbleGoal: //put down wobble goal
						newState(MainState.state2);
						break;
					case state2: //moves to waypoint 2 in front of second powershot behind launch line
						robot.strafe(org.firstinspires.ftc.teamcode.Autonomous.AutoUtils.Utils.convertInches2Ticks(46), 255, 225, 1, 0, 0);
						if (robot.isStrafeComplete) {
							newState(MainState.state2Turn);
						}
						break;
					case state2Turn: //turn towards center powershot
						robot.turn(0, 1, 1);
						if (robot.isTurnComplete) {
							newState(MainState.statePS1);

						}
						break;
					case statePS1: //do powershot shooter code
//						if (shooter.feederCount() >= 1) {
//							//do turn to next powershot
//							robot.turn();
//							if (robot.isTurnComplete) {
//								newState(MainState.statePS2);
//								break;
//							}
//							break;
//
//						}
//
//						robot.setPower(0, 0, 0, 0);
//						shooter.powerShot();
//
//						if (mainTime.seconds() > 1) {
//							shooter.feederState(true);
//						}
						break;
//					case statePS2:
//						if (shooter.feederCount() >= 1) {
//							//do turn to next powershot
//							robot.turn();
//							if (robot.isTurnComplete) {
//								newState(MainState.statePS3);
//								break;
//							}
//							break;
//
//						}
//
//						robot.setPower(0, 0, 0, 0);
//						shooter.powerShot();
//
//						if (mainTime.seconds() > 1) {
//							shooter.feederState(true);
//						}
//						break;
//					case statePS3:
//						if (shooter.feederCount() >= 1) {
//							//do turn to 0 degrees
//							robot.turn();
//							if (robot.isTurnComplete) {
//								newState(MainState.state3);
//								break;
//							}
//							break;
//
//						}
//
//						robot.setPower(0, 0, 0, 0);
//						shooter.powerShot();
//
//						if (mainTime.seconds() > 1) {
//							shooter.feederState(true);
//						}
//						break;
//					case state3: //move to waypoint3
//						robot.strafe(2000, 0, 0, 1, 0, 0);
//						if (robot.isStrafeComplete) {
//							newState(MainState.state4);
//						}
//						break;
//					case state4: //move forward to fourth wobble goal position
//						robot.strafe(2000, 0, 0, 1, 0, 0);
//						if (robot.isStrafeComplete) {
//							newState(MainState.state4Turn);
//						}
//						break;
//					case state4Turn: //turn
//						robot.turn();
//						if (robot.isTurnComplete) {
//							newState(MainState.state4WobbleGoal);
//						}
//						break;
//					case state4WobbleGoal: //put down wobble goal
//						break;
//

				}

				break;

			case 1:
				break;

			case 4:
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
		state4,
		state4Turn,
		state4WobbleGoal

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
