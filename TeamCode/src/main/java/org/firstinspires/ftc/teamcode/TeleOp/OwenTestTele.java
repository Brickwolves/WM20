/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "TEST Tele", group = "TeleOp")
//@Disabled
public class OwenTestTele extends OpMode {
	
	private boolean angleOffset = false;
	private boolean timeStop = true;
	
	private Controller driver, operator;
	private Controller.Thumbstick driverRightStick, driverLeftStick;
	
	private ElapsedTime mainTime = new ElapsedTime();
	
	
	private Gyro gyro;
	private MecanumChassis robot;
	private Shooter shooter;
	private Intake intake;
	private WobbleGripper wobble;
	
	@Override
	public void init() {
		telemetry.addData("Status", "Initialized");
		
		Servo feeder = hardwareMap.get(Servo.class, "feeder");
		Servo feederLock = hardwareMap.get(Servo.class, "feederlock");
		
		Servo reachOne = hardwareMap.get(Servo.class, "outerrollerone");
		Servo reachTwo = hardwareMap.get(Servo.class, "outerrollertwo");
		
		Servo lifter = hardwareMap.get(Servo.class, "lifter");
		Servo gripperOne = hardwareMap.get(Servo.class, "gripperone");
		Servo gripperTwo = hardwareMap.get(Servo.class, "grippertwo");
		
		driver = new Controller(gamepad1);
		operator = new Controller(gamepad2);
		driverRightStick = driver.getRightThumbstick();
		driverLeftStick = driver.getLeftThumbstick();
		driverLeftStick.setShift(0);
		
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
		robot = new MecanumChassis(frontLeft, frontRight, backLeft, backRight, gyro);
		shooter = new Shooter(shooterOne, shooterTwo, feeder, feederLock);
		intake = new Intake(intakeDrive, reachOne, reachTwo);
		wobble = new WobbleGripper(gripperOne, gripperTwo, lifter);
	}
	
	
	@Override
	public void init_loop() {
		shooter.lockFeeder();
		shooter.resetFeeder();
		shooter.shooterOff();
		intake.intakeOff();
		
		if(operator.RSToggle()){
			intake.setBumperThreshold(3);
		}else{
			intake.retractBumper();
		}
		
		angleOffset = driver.crossToggle();
		timeStop = !driver.squareToggle();
		robot.setPower(0,0, gamepad1.left_stick_x*-1, .5);
		
		telemetry.addData("angle offset", angleOffset);
		telemetry.addData("time stop", timeStop);
		telemetry.update();
	}
	
	
	@Override
	public void start() {
		if(angleOffset){
			robot.resetGyro(90);
		}else{
			robot.resetGyro(0);
		}
		if(timeStop) {
			wobble.newState(WobbleGripper.ArmState.STATE_DOWN);
			wobble.newState(WobbleGripper.GripperState.STATE_GRIP);
		}
		mainTime.reset();
	}
	
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void loop() {
		Controller.Thumbstick driverRightStick = driver.getRightThumbstick();
		Controller.Thumbstick driverLeftStick = driver.getLeftThumbstick();
		Controller.Thumbstick operatorRightStick = operator.getRightThumbstick();
		Controller.Thumbstick operatorLeftStick = operator.getLeftThumbstick();

		driverRightStick.setShift(gyro.getModAngle());
		driverLeftStick.setShift(0);
		operatorRightStick.setShift(0);
		operatorLeftStick.setShift(0);
		
		boolean intakeOff = driver.trianglePressUpdate() || (driver.leftPressUpdate() || driver.rightPressUpdate());
		boolean shooterOff = driver.crossPressUpdate() || driver.circlePressUpdate();
		
		
		//driver controls
		robot.driveState(driverRightStick.getInvertedShiftedY(), driverRightStick.getInvertedShiftedX(),
				driverLeftStick.getInvertedShiftedX(), driver.RT());
		robot.setCardinalAngle(driver.upPressUpdate(), driver.rightPressUpdate(), driver.downPressUpdate(), driver.leftPressUpdate());
		robot.adjustmentState(driver.RBPressUpdate(), driver.LBPressUpdate(), 10);
		
		
		//operator controls
		shooter.shooterState(driver.trianglePress(), driver.trianglePress() || shooterOff, driver.LT() > .5, false);
		shooter.feederState(driver.square());
		
		intake.intakeState(driver.crossPress(), driver.crossPress() || intakeOff, driver.circle());
		intake.reachState(driver.RSPressUpdate());
		
		
		//telemetry
		telemetry.addData("time", mainTime.seconds());
		telemetry.addData("shooter rpm", shooter.getRPM());
		/*telemetry.addData("autodrive", robot.autoDrive);
		telemetry.addData("autostrafe", robot.autoStrafe);
		telemetry.addData("autoturn", robot.autoTurn);
		telemetry.addData("autopower", robot.autoPower);*/
		telemetry.update();
		
		
		if(mainTime.seconds() > 87 && mainTime.seconds() < 88 && timeStop){
			wobble.newState(WobbleGripper.ArmState.STATE_UP);
		}
		
		
		if(timeStop && mainTime.seconds() > 121) {
			requestOpModeStop();
		}
		
		if(driver.RS() && driver.LS()){
			requestOpModeStop();
		}
		
		
		
	}
	
}