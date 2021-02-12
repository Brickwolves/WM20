//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import android.os.Build;
//
//import androidx.annotation.RequiresApi;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Autonomous.AutoUtils.DashConstants;
//import org.firstinspires.ftc.teamcode.Autonomous.AutoUtils.DashVision;
//import org.firstinspires.ftc.teamcode.HardwareClasses.Gyro;
//import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
//import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumChassis;
//import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
//import org.firstinspires.ftc.utilities.IMU;
//import org.firstinspires.ftc.utilities.Utils;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@Autonomous(name = "Owen Kinky Auto", group = "Auto")
//@Disabled
//public class OwenKinkyAuto extends OpMode {
//
//	private final ElapsedTime feederTime = new ElapsedTime();
//	private final ElapsedTime mainTime = new ElapsedTime();
//
//	private Gyro gyro;
//	private MecanumChassis robot;
//	private Shooter shooter;
//	private Intake intake;
//	boolean isFeederLocked = true;
//
//	private MainState currentMainState = MainState.STATE_FORWARD;
//	OpenCvCamera webcam;
//
//	private FtcDashboard dashboard = FtcDashboard.getInstance();
//	private Telemetry dashboardTelemetry = dashboard.getTelemetry();
//	private MultipleTelemetry multTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
//
//
//	private static double ringCount = 0;
//	private boolean ringsFound = false;
//
//
//	@Override
//	public void init() {
//		Servo feeder = hardwareMap.get(Servo.class, "feeder");
//		Servo feederLock = hardwareMap.get(Servo.class, "feederlock");
//
//		Servo outerRollerOne = hardwareMap.get(Servo.class, "outerrollerone");
//		Servo outerRollerTwo = hardwareMap.get(Servo.class, "outerrollertwo");
//
//		DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
//		DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontright");
//		DcMotor backLeft = hardwareMap.get(DcMotor.class, "backleft");
//		DcMotor backRight = hardwareMap.get(DcMotor.class, "backright");
//
//		DcMotor shooterOne = hardwareMap.get(DcMotor.class, "shooterone");
//		DcMotor shooterTwo = hardwareMap.get(DcMotor.class, "shootertwo");
//		DcMotor intakeDrive = hardwareMap.get(DcMotor.class, "intake");
//
//		Utils.setHardwareMap(hardwareMap);
//		IMU imu = new IMU("imu");
//		gyro = new Gyro(imu, 0);
//		shooter = new Shooter(shooterOne, shooterTwo, feeder, feederLock);
//		intake = new Intake(intakeDrive, outerRollerOne,outerRollerTwo);
//		robot = new MecanumChassis(frontLeft, frontRight, backLeft, backRight, gyro);
//
//
//	}
//
//	public void init_loop(){
//		intake.retractReach();
//		intake.intakeOff();
//		shooter.resetFeeder();
//		shooter.lockFeeder();
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
//	}
//
//	public void start() {
//		mainTime.reset();
//		robot.resetGyro();
//		robot.resetMotors();
//	}
//
//	@RequiresApi(api = Build.VERSION_CODES.N)
//	@Override
//	public void loop() {
//		switch (ringCount) {
//			case 0:
//				switch (currentMainState) {
//					case state1: //move forward to first wobble goal position
//						robot.strafe(2000,0,0,1,0,0);
//						if(robot.isStrafeComplete){
//							newState(MainState.state1Turn);
//						}
//						break;
//					case state1Turn: //turn
//						robot.turn();
//						if(robot.isTurnComplete){
//							newState(MainState.state1WobbleGoal);
//						}
//						break;
//					case state1WobbleGoal: //put down wobble goal
//						break;
//					case state2: //moves to waypoint 2 in front of second powershot behind launch line
//						robot.strafe(2000,0,0,1,0,0);
//						if(robot.isStrafeComplete){
//							newState(MainState.state2Turn);
//						}
//						break;
//					case state2Turn: //turn towards center powershot
//						robot.turn();
//						if(robot.isTurnComplete){
//							newState(MainState.statePS1);
//						}
//						break;
//					case statePS1: //do powershot shooter code
//						if (shooter.feederCount() >= 1) {
//							newState(MainState.statePS1Turn);
//							break;
//						}
//
//						robot.setPower(0,0,0,0);
//						shooter.powerShot();
//
//						if(mainTime.seconds()>1){
//							shooter.feederState(true);
//						}
//						break;
//					case: statePS1Turn
//					case state3:
//						break;
//					case state4:
//						break;
//					case state5:
//						break;
//					case state6:
//						break;
//					case state7:
//						break;
//					case state8:
//						break;
//
//				}
//
//				break;
//
//			case 1:
//				break;
//
//			case 4:
//				break;
//		}
//		switch (currentMainState) {
//			case STATE_FORWARD:
//				robot.strafe(2000,0,0,1,0,0);
//				intake.deployReach();
//
//				if(robot.currentTicks>750) {
//
//					intake.intakeOn();
//				}
//				if(robot.isStrafeComplete){
//					newState(MainState.STATE_LINEUP);
//				}
//				break;
//
//			case STATE_LINEUP:
//				robot.strafe(400,0,180,.5,0,0);
//				if(robot.currentTicks < 350){ intake.intakeOn(); }
//				else{ intake.intakeOff(); intake.retractReach(); }
//
//				if(robot.isStrafeComplete){
//					newState(MainState.STATE_SHOOT);
//				}
//				break;
//
//			case STATE_SHOOT:
//				if (shooter.feederCount() >= 3) {
//					newState(MainState.STATE_STRAFE);
//					break;
//				}
//
//				robot.setPower(0,0,0,0);
//				intake.retractReach();
//				intake.intakeOff();
//				shooter.topGoal();
//
//				if(mainTime.seconds()>1){
//					shooter.feederState(true);
//				}
//				break;
//
//			case STATE_STRAFE:
//				shooter.shooterOff();
//		}
//
//		telemetry.addData("flywheel rpm = ", Math.abs(shooter.updateRPM()));
//
//
//	}
//
//
//
//
//	private void newState(MainState newState) {
//		currentMainState = newState;
//		mainTime.reset();
//	}
//
//	private enum MainState {
//		STATE_IDK_I_GIVE_UP,
//		state2
//	}
//
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
//}
