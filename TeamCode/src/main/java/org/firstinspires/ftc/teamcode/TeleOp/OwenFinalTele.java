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
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Gyro;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.Katana;
import org.firstinspires.ftc.teamcode.HardwareClasses.MecanumChassis;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.WobbleGripper;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.Utils;

@TeleOp(name = "Owen Final Tele", group = "TeleOp")
//@Disabled
public class OwenFinalTele extends OpMode {
	
	private boolean realMatch = true;
	private int ringCount;
	
	private Controller driver, operator;
	private Controller.Thumbstick driverRightStick, driverLeftStick;
	
	private ElapsedTime mainTime = new ElapsedTime();
	
	
	private Gyro gyro;
	private MecanumChassis robot;
	private Shooter shooter;
	private Intake intake;
	private WobbleGripper wobble;
	private Katana katana;
	private Sensors sensors;
	
	@Override
	public void init() {
		telemetry.addData("Status", "Initialized");
		
		Servo feeder = hardwareMap.get(Servo.class, "feeder");
		Servo feederLock = hardwareMap.get(Servo.class, "feederlock");
		
		Servo bumperLeft = hardwareMap.get(Servo.class, "bumperleft");
		Servo bumperRight = hardwareMap.get(Servo.class, "bumperright");
		
		Servo lifter = hardwareMap.get(Servo.class, "lifter");
		Servo gripperOne = hardwareMap.get(Servo.class, "gripperone");
		Servo gripperTwo = hardwareMap.get(Servo.class, "grippertwo");
		
		Servo katanaLeft = hardwareMap.get(Servo.class, "katanaleft");
		Servo katanaRight = hardwareMap.get(Servo.class, "katanaright");
		
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
		
		robot = new MecanumChassis(frontLeft, frontRight, backLeft, backRight);
		sensors = new Sensors(imu, null, null,null, null);
		
		shooter = new Shooter(shooterOne, shooterTwo, feeder, feederLock);
		intake = new Intake(intakeDrive, bumperLeft, bumperRight);
		wobble = new WobbleGripper(gripperOne, gripperTwo, lifter);
		katana = new Katana(katanaLeft, katanaRight);
		
		mainTime.reset();
	}
	
	
	@Override
	public void init_loop() {
		shooter.lockFeeder();
		shooter.resetFeeder();
		shooter.shooterOff();
		intake.intakeOff();
		katana.katanaUp();
		
		driver.update();
		
		if(wobble.lifter.getPosition() > .5){
			ringCount = 4;
			wobble.armFold();
			wobble.gripperHalf();
		}else if(intake.bumperLeft.getPosition() < .2){
			ringCount = 1;
			wobble.armDown();
			if(mainTime.seconds() > .4){ wobble.gripperGrip(); }
		}else{
			ringCount = 0;
			wobble.armDown();
			if(mainTime.seconds() > .4){ wobble.gripperGrip(); }
		}
		
		realMatch = !driver.crossToggle();
		
		robot.setPower(0,0, gamepad1.left_stick_x*-1, (((driver.RTFloat() + 1) / -2) + 1.5) * .6);
		
		telemetry.addData("bumper position", intake.bumperLeft.getPosition());
		telemetry.addData("ring count", ringCount);
		telemetry.addData("real match", realMatch);
		telemetry.update();
	}
	
	
	@Override
	public void start() {
		if(realMatch){
			switch((int) ringCount){
				case 0:
					wobble.newState(WobbleGripper.GripperState.STATE_GRIP);
					wobble.newState(WobbleGripper.ArmState.STATE_FOLD);
					robot.resetGyro(0);
					break;
				
				case 1:
					wobble.newState(WobbleGripper.GripperState.STATE_GRIP);
					wobble.newState(WobbleGripper.ArmState.STATE_FOLD);
					robot.resetGyro(90);
					break;
				
				case 4:
					wobble.newState(WobbleGripper.GripperState.STATE_HALF);
					wobble.newState(WobbleGripper.ArmState.STATE_FOLD);
					robot.resetGyro(0);
					break;
			}
		}else{
			robot.resetGyro(0);
		}
		
		mainTime.reset();
	}
	
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void loop() {
		Controller.Thumbstick driverRightStick = driver.getRightThumbstick();
		Controller.Thumbstick driverLeftStick = driver.getLeftThumbstick();

		driverRightStick.setShift(Sensors.gyro.getModAngle());
		driverLeftStick.setShift(0);
		
		
		driver.update();
		operator.update();
		
		//driver controls
		robot.driveState(driverRightStick.shiftedY(), driverRightStick.shiftedX(), driverLeftStick.shiftedX(), driver.RTFloat());
		robot.setCardinalAngle(driver.upPress(), driver.rightPress(), driver.downPress(), driver.leftPress(), false);
		robot.adjustmentState(driver.RBPress(), driver.LBPress(), 10);
		
		
		//operator controls
		shooter.shooterState(operator.trianglePress(), operator.leftPress(), operator.rightPress());
		shooter.feederState(operator.square());
		
		intake.intakeState(operator.crossPress(), operator.circle());
		intake.bumperState(operator.RSPress(), driver.LT());
		
		wobble.armState(operator.LBPress(), operator.LSPress());
		wobble.gripperState(operator.RBPress());
		
		katana.katanaState(operator.LTToggle(), operator.RT());
		
		
		//telemetry
		telemetry.addData("time", mainTime.seconds());
		telemetry.addData("shooter rpm", shooter.getRPM());
		telemetry.addData("mod angle", Sensors.gyro.getModAngle());
		telemetry.update();
		
		if(driver.squarePress()){
			robot.resetGyro(-90);
		}
		
		
		if(mainTime.seconds() > 87 && mainTime.seconds() < 88 && realMatch){
			wobble.newState(WobbleGripper.ArmState.STATE_UP);
		}
		
		
		if((realMatch && mainTime.seconds() > 121) || driver.touchpad()) {
			requestOpModeStop();
		}
		
	}
	
}