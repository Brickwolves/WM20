/* Copyright (c) 2017 FIRST. All rights reserved. * * Redistribution and use in source and binary forms, with or without modification, * are permitted (subject to the limitations in the disclaimer below) provided that * the following conditions are met: * * Redistributions of source code must retain the above copyright notice, this list * of conditions and the following disclaimer. * * Redistributions in binary form must reproduce the above copyright notice, this * list of conditions and the following disclaimer in the documentation and/or * other materials provided with the distribution. * * Neither the name of FIRST nor the names of its contributors may be used to endorse or * promote products derived from this software without specific prior written permission. * * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */package org.firstinspires.ftc.teamcode.TeleOp;import android.os.Build;import androidx.annotation.RequiresApi;import com.qualcomm.robotcore.eventloop.opmode.OpMode;import com.qualcomm.robotcore.eventloop.opmode.TeleOp;import com.qualcomm.robotcore.util.ElapsedTime;import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;import org.firstinspires.ftc.teamcode.HardwareClasses.FancyController;import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;import org.firstinspires.ftc.teamcode.HardwareClasses.Katana;import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;import org.firstinspires.ftc.teamcode.HardwareClasses.Wobble;import org.firstinspires.ftc.utilities.Utils;@TeleOp(name = "Clean TELE OP", group = "TeleOp")public class CleanTeleOp extends OpMode {		private boolean realMatch = true, autoAim = true;	private int gyroOffset = 0;	private String offsetTelemetry = "forward";		private final ElapsedTime mainTime = new ElapsedTime();	private Controller driver, operator;		private FancyController drivertwo;			@Override	public void init() {				Utils.setHardwareMap(hardwareMap);		Robot.init(); Sensors.init(); Shooter.init(); Intake.init(); Wobble.init(); Katana.init();				drivertwo = new FancyController(gamepad1); operator = new Controller(gamepad2);		mainTime.reset();				drivertwo.RT.value();				Sensors.frontCamera.setPipeline(Sensors.frontCamera.autoAimPipeline);		Sensors.frontCamera.startVision(320, 240);	}			@RequiresApi(api = Build.VERSION_CODES.N)	@Override	public void init_loop() {		Sensors.update();		driver.update(); operator.update();				Robot.setPower(0,0, driver.leftStick.X(), driver.RTRange(.3, .6));		Shooter.lockFeeder(); Shooter.resetFeeder(); Shooter.shooterOff();		Intake.intakeOff();				if(driver.upPress()) { gyroOffset = 0; offsetTelemetry = "forward"; }		if(driver.leftPress()) { gyroOffset = -90; offsetTelemetry = "left"; }		if(driver.rightPress()) { gyroOffset = 90; offsetTelemetry = "right"; }		if(driver.downPress()) { gyroOffset = 180; offsetTelemetry = "backward"; }				Shooter.setTurretAngle(0);				realMatch = !driver.shareToggle();		autoAim = !driver.triangleToggle();				initTelemetry();	}			@Override	public void start() {		Sensors.update();		randomizationInit();		Wobble.newState(Wobble.GripperState.HALF); Wobble.newState(Wobble.ArmState.FOLD);		mainTime.reset();	}			@RequiresApi(api = Build.VERSION_CODES.N)	@Override	public void loop() {		Sensors.update(); driver.update(); operator.update();		driver.rightStick.setShift(Sensors.gyro.modAngle());				//movement controls		Robot.driveState(driver.rightStick.shiftedY(), driver.rightStick.shiftedX(), driver.leftStick.X(), driver.RTRange(.5, 1));		Robot.cardinalState(driver.upPress(), driver.rightPress(), driver.downPress(), driver.leftPress());		Robot.autoAimState(driver.RSPress(), autoAim, driver.circlePress(), driver.crossPress(), 9);				autoAim = !driver.triangleToggle();		realMatch = !driver.shareToggle();		if(driver.squarePress()){ Robot.resetGyro(-90); }								//operator controls		Shooter.shooterState(operator.trianglePress(), operator.leftPress(), operator.rightPress(), autoAim);		Shooter.feederState(operator.square());				Intake.intakeState(operator.crossPress(), operator.circle(), driver.LB());		Intake.fabricState(operator.RSPress(), driver.LT());				Wobble.armState(operator.LBPress(), operator.LSPress());		Wobble.gripperState(operator.RBPress());				Katana.katanaState(!driver.RBToggle(), operator.LT());				//match timers		if(mainTime.seconds() > 87 && mainTime.seconds() < 87.5 && realMatch) Wobble.newState(Wobble.ArmState.UP);		if(mainTime.seconds() > 87.5 && mainTime.seconds() < 88 && realMatch) Wobble.newState(Wobble.ArmState.FOLD);		if((realMatch && mainTime.seconds() > 121) || driver.touchpad()) requestOpModeStop();				loopTelemetry();	}					//TELEMETRY		private void initTelemetry(){		telemetry.addData("auto aim", autoAim);		telemetry.addData("real match", realMatch);		telemetry.addData("gyro offset", gyroOffset);		telemetry.addData("start angle", offsetTelemetry);		telemetry.update();	}		private void loopTelemetry(){		telemetry.addData("time", mainTime.seconds());		telemetry.addData("auto aim", autoAim);		telemetry.addData("real match", realMatch);		telemetry.addLine();				telemetry.addLine("// DRIVE TELEMETRY //");		telemetry.addData("MEASURED angle", Sensors.gyro.modAngle());		telemetry.addData("robtt velocity component", Sensors.robotVelocityComponent(Sensors.frontCamera.towerAimError() - 90));		telemetry.addLine();				telemetry.addLine("// SHOOTER TELEMETRY //");		telemetry.addData("target rpm", Shooter.targetRPM);		telemetry.addData("measured rpm", Shooter.getRPM());		telemetry.addData("min rpm", Shooter.minRPM);		telemetry.addData("turret angle", Shooter.getTurretAngle());		telemetry.addData("is ring loaded", Sensors.isRingLoaded());		telemetry.addLine();				telemetry.addLine("// VISION TELEMETRY //");		telemetry.addData("tower aim error", Sensors.frontCamera.towerAimError());		telemetry.addData("distance to goal", Sensors.frontCamera.towerDistance());		telemetry.addData("shooter offset angle", Sensors.frontCamera.shooterOffsetAngle());		telemetry.addLine();				telemetry.addLine("// INTAKE TELEMETRY //");		telemetry.addData("target rpm", Intake.targetRPM);		telemetry.addData("measured rpm", Intake.getRPM());		telemetry.addData("power", Intake.getPower());		telemetry.update();	}		//RANDOMIZATION EVAL		private void randomizationEval(){		if(Wobble.gripperPosition() > .48){			gyroOffset = 0;		}else if(Wobble.gripperPosition() < .15){			gyroOffset = 90;		}else{			gyroOffset = -90;		}	}		private void randomizationInit(){		Robot.resetGyro(gyroOffset);	}}