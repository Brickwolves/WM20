/* Copyright (c) 2017 FIRST. All rights reserved. * * Redistribution and use in source and binary forms, with or without modification, * are permitted (subject to the limitations in the disclaimer below) provided that * the following conditions are met: * * Redistributions of source code must retain the above copyright notice, this list * of conditions and the following disclaimer. * * Redistributions in binary form must reproduce the above copyright notice, this * list of conditions and the following disclaimer in the documentation and/or * other materials provided with the distribution. * * Neither the name of FIRST nor the names of its contributors may be used to endorse or * promote products derived from this software without specific prior written permission. * * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */package org.firstinspires.ftc.teamcode.TeleOp;import android.os.Build;import androidx.annotation.RequiresApi;import com.qualcomm.robotcore.eventloop.opmode.OpMode;import com.qualcomm.robotcore.eventloop.opmode.TeleOp;import com.qualcomm.robotcore.util.ElapsedTime;import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;import org.firstinspires.ftc.teamcode.HardwareClasses.Katana;import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;import org.firstinspires.ftc.teamcode.HardwareClasses.Wobble;import org.firstinspires.ftc.utilities.Utils;@TeleOp(name = "Clean TELE OP", group = "TeleOp")public class CleanTeleOp extends OpMode {		private boolean realMatch = true, autoAim = true;	private int ringCount;		private final ElapsedTime mainTime = new ElapsedTime();	private Controller driver, operator;			@Override	public void init() {				Utils.setHardwareMap(hardwareMap);		Robot.init(); Sensors.init(); Shooter.init(); Intake.init(); Wobble.init(); Katana.init();				driver = new Controller(gamepad1); operator = new Controller(gamepad2);		mainTime.reset();				Sensors.frontCamera.setPipeline(Sensors.frontCamera.autoAimPipeline);		Sensors.frontCamera.startVision(320, 240);	}			@RequiresApi(api = Build.VERSION_CODES.N)	@Override	public void init_loop() {		Sensors.update();		driver.update(); operator.update();		randomizationEval();				Robot.setPower(0,0, driver.leftStick.X(), driver.RTRange(.3, .6));		Shooter.lockFeeder(); Shooter.resetFeeder(); Shooter.shooterOff();		Intake.intakeOff();				if(operator.squareToggle()) Shooter.setTurretAngle(-22.5);		else Shooter.setTurretAngle(38);				realMatch = !driver.shareToggle();		autoAim = !driver.triangleToggle();				initTelemetry();	}			@Override	public void start() {		Sensors.update();		if(realMatch) randomizationInit();		else{ Wobble.newState(Wobble.GripperState.HALF); Robot.resetGyro(0); }		mainTime.reset();	}			@RequiresApi(api = Build.VERSION_CODES.N)	@Override	public void loop() {		Sensors.update(); driver.update(); operator.update();		driver.rightStick.setShift(Sensors.gyro.modAngle());				Shooter.turretAim();				//movement controls		Robot.driveState(driver.rightStick.shiftedY(), driver.rightStick.shiftedX(), driver.leftStick.X(), driver.RTRange(.5, 1));		Robot.cardinalState(driver.upPress(), driver.rightPress(), driver.downPress(), driver.leftPress());		Robot.autoAimState(driver.RSPress(), autoAim, driver.circlePress(), driver.crossPress(), 9);				autoAim = !driver.triangleToggle();		realMatch = !driver.shareToggle();		if(driver.squarePress()){ Robot.resetGyro(-90); }								//operator controls		Shooter.shooterState(operator.trianglePress(), operator.leftPress(), operator.rightPress(), autoAim);		Shooter.feederTeleState(operator.square(), operator.RT());				Intake.intakeState(operator.crossPress(), operator.circle(), driver.LB());		Intake.fabricState(operator.RSPress(), driver.LT());				Wobble.armState(operator.LBPress(), operator.LSPress());		Wobble.gripperState(operator.RBPress());				Katana.katanaState(driver.RBToggle());				//match timers		if(mainTime.seconds() > 87 && mainTime.seconds() < 88 && realMatch) Wobble.newState(Wobble.ArmState.UP);		if((realMatch && mainTime.seconds() > 121) || driver.touchpad()) requestOpModeStop();				loopTelemetry();	}					//TELEMETRY		private void initTelemetry(){		telemetry.addData("auto aim", autoAim);		telemetry.addData("real match", realMatch);		telemetry.addData("ring count", ringCount);		telemetry.update();	}		private void loopTelemetry(){		telemetry.addData("time", mainTime.seconds());		telemetry.addData("auto aim", autoAim);		telemetry.addData("real match", realMatch);		telemetry.addData("MEASURED angle", Sensors.gyro.modAngle());		telemetry.addData("robtt velocity component", Sensors.robotVelocityComponent(Sensors.frontCamera.towerAimError() - 90));		telemetry.addData("", null);		telemetry.addData("hopper red", Sensors.hopperColor.red());		telemetry.addData("hopper hue", Sensors.hopperColor.hue());		telemetry.addData("is ring loaded", Sensors.isRingLoaded());		telemetry.addData("", null);		telemetry.addData("TARGET rpm", Shooter.targetRPM);		telemetry.addData("MEASURED rpm", Shooter.getRPM());		telemetry.addData("", null);		telemetry.addData("turret angle", Shooter.getTurretAngle());		telemetry.addData("vertical component", Shooter.verticalComponent());		telemetry.addData("", null);		telemetry.addData("tower aim error", Sensors.frontCamera.towerAimError());		telemetry.addData("distance to goal", Sensors.frontCamera.towerDistance());		telemetry.addData("shooter offset angle", Sensors.frontCamera.shooterOffsetAngle());		telemetry.update();	}		//RANDOMIZATION EVAL		private void randomizationEval(){		if(Wobble.lifter.getPosition() > .5){			ringCount = 4;			Wobble.armFold();			Wobble.gripperHalf();		}else if(Intake.fabricPosition < .2){			ringCount = 1;			Wobble.armDown();			if(mainTime.seconds() > .4) Wobble.gripperGrip();		}else{			ringCount = 0;			Wobble.armDown();			if(mainTime.seconds() > .4) Wobble.gripperGrip();		}	}		private void randomizationInit(){		switch(ringCount){			case 0:				Wobble.newState(Wobble.GripperState.GRIP);				Robot.resetGyro(0);				break;			case 1:				Wobble.newState(Wobble.GripperState.GRIP);				Robot.resetGyro(90);				break;			case 4:				Wobble.newState(Wobble.GripperState.HALF);				Robot.resetGyro(0);				break;		}	}}