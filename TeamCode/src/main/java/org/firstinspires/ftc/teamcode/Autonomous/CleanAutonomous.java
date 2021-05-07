package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.Katana;
import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.Wobble;
import org.firstinspires.ftc.utilities.Utils;

import static android.os.SystemClock.sleep;

@Autonomous(name = "Clean Auto", group = "Auto")

public class CleanAutonomous extends OpMode {
	
	private Controller operator;
	
	
	private MainState currentMainState = MainState.state1Drive;
	private final ElapsedTime mainTime = new ElapsedTime();
	

	private static double ringCount = 0;
	private final boolean ringsFound = false;

	@Override
	public void init() {
		
		Utils.setHardwareMap(hardwareMap);
		Robot.init(); Sensors.init(); Shooter.init(); Intake.init(); Wobble.init(); Katana.init();
		
		operator = new Controller(gamepad2);
		
		Sensors.backCamera.setPipeline(Sensors.backCamera.ringFinderPipeline);
		Sensors.backCamera.startVision(1920, 1080);
	}
	
	public void init_loop(){
		operator.update();
		Shooter.resetFeeder();
		Shooter.lockFeeder();
		
		if(operator.RBToggle()) Wobble.gripperGrip();
		else Wobble.gripperHalf();
		
		if(operator.crossToggle()){
			Intake.intakeStallControl();
			Intake.fabricGroundRings();
		}else{
			Intake.intakeOff();
			Intake.fabricRetract();
		}
		
		telemetry.addData("Ring Count = ", Sensors.backCamera.startingStackCount());
		telemetry.update();
		
		sleep(50);
	}
	
	public void start() {
		Sensors.update();
		mainTime.reset();
		Robot.resetGyro(90);
		Robot.resetWithoutEncoders();
		ringCount = Sensors.backCamera.startingStackCount();
		Sensors.backCamera.stopVision();
		Sensors.frontCamera.startVision(320, 240);
		Sensors.frontCamera.setPipeline(Sensors.frontCamera.autoAimPipeline);
		Shooter.setFeederCount(0);
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void loop() {
		Sensors.update(); operator.update();
		Katana.katanaAutoState();
		Shooter.setTurretAngle(0);
		
		switch ((int) ringCount) {
			case 0:
				switch (currentMainState) {
					//drive to target A
					case state1Drive:
						Robot.strafe(33.5, 180, 0, 1, 0, 0);
						
						if (Robot.currentInches > 18) Wobble.armDown();
						
						if (Robot.isStrafeComplete) {
							Robot.setPower(0, 0, 0, 1);
							newState(MainState.state2WobbleGoal);
						}
						
						break;
					
					//put down wobble goal and back up
					case state2WobbleGoal:
						Wobble.armDown();
						if (mainTime.seconds() > .9) {
							Wobble.gripperOpen();
							Robot.strafe(6, 180, 180, .5, 0, 0);
						} else Robot.setPower(0, 0, 0, 1);
						
						
						if (mainTime.seconds() > .9 && Robot.isStrafeComplete) {
							newState(MainState.state3Turn);
							Wobble.armFold();
							Wobble.gripperHalf();
						}
						break;
					
					
					//turn towards power shot shooting position
					case state3Turn: //turn
						Robot.turn(270, 1, 0);
						
						if (Robot.isTurnComplete) newState(MainState.state4Drive);
						
						break;
					
					
					//drive to power shot shooting position
					case state4Drive:
						Robot.strafe(29.5, 270, 270, 1, 0, 0);
						
						if (Robot.isStrafeComplete) newState(MainState.state5Turn);
						
						break;
					
					
					//turn to face powershots
					case state5Turn:
						if (mainTime.seconds() > .7) {
							Robot.turn(-5, 1, 0);
							Shooter.powerShot();
						}
						
						if (mainTime.seconds() > .7 && Robot.isTurnComplete) newState(MainState.state6PS1);
						break;
					
					
					case state6PS1:
						Shooter.powerShot();
						
						if (Shooter.feederCount() < 1) {
							Robot.setPowerAuto(0, 0, 0);
							if (mainTime.seconds() > .8) Shooter.feederState(true, 1.5);
						} else {
							Robot.turn(6, 1, 1);
							if (mainTime.seconds() > 1.7 && Robot.isTurnComplete) newState(MainState.state7PS2);
						}
						break;
					
					
					//shoot power shot 2, turn to power shot 3
					case state7PS2:
						Shooter.powerShot();
						if (Shooter.feederCount() < 2) {
							Robot.setPowerAuto(0, 0, 6);
							if (mainTime.seconds() > .2) Shooter.feederState(true,1.5);
							
						} else {
							Robot.turn(11.3, 1, 1);
							
							if (Robot.isTurnComplete) newState(MainState.state8PS3);
							
						}
						break;
					
					
					//shoot power shot 3, turn to second wobble goal
					case state8PS3:
						Shooter.powerShot();
						Wobble.armDown();
						Wobble.gripperOpen();
						
						if (Shooter.feederCount() < 3) {
							if (mainTime.seconds() > 0) Shooter.feederState(true, 1.5);
							Robot.setPowerAuto(0, 0, 11.3);
						} else {
							Robot.turn(-37, 1, .8);
							Shooter.shooterOff();
							
							if (Robot.isTurnComplete) newState(MainState.state9Drive);
						}
						break;
					
					
					//drive to second wobble goal
					case state9Drive:
						Robot.strafe(14, -37, 143, 1, 0, 0);
						if (Robot.isStrafeComplete) {
							newState(MainState.state10WobbleGoal);
							Robot.setPower(0, 0, 0, 1);
						}
						break;
					
					
					//pick up second wobble goal, turn away from target A
					case state10WobbleGoal:
						Wobble.armDown();
						
						if (mainTime.seconds() > .8) Wobble.gripperGrip();
						
						if (mainTime.seconds() > 1.6) {
							Robot.turn(187, 1, 0);
							Wobble.armPosition(.15);
							if (Robot.isTurnComplete) newState(MainState.state11Drive);
						}
						break;
					
					
					//drive to target A
					case state11Drive:
						Wobble.armPosition(.15);
						Wobble.gripperGrip();
						Robot.strafe(22, 187, 7, 1, 0, 0);
						
						if (Robot.isStrafeComplete) newState(MainState.state13Turn);
						break;
						
					
					case state13Turn:
						Wobble.gripperGrip();
						Wobble.armPosition(.15);
						if (mainTime.seconds() > .7) Robot.turn(-85, 1, .5);
						if (mainTime.seconds() > 1 && Robot.isTurnComplete) newState(MainState.state12WobbleGoal);
						break;
						
						
					//put down second wobble goal and back up
					case state12WobbleGoal:
						Wobble.armPosition(.15);
						Wobble.gripperGrip();
						Robot.strafe(10, -90, 90, .6, .2, 0);
						if(Robot.isStrafeComplete)newState(MainState.stateFinished);
						break;
						
					case stateFinished:
						Robot.setPowerAuto(0,0,-90,2);
				}
				
				break;
			
			case 1:
				switch (currentMainState) {
					//drive to WP1
					case state1Drive:
						Robot.strafe(40, 180, 0, 1, .2, 0);
						
						if (Robot.isStrafeComplete) newState(MainState.state1Turn);
						break;
					
					
					case state1Turn:
						if (mainTime.seconds() > .7) Robot.turn(135, 1, .5);
						if (mainTime.seconds() > .9 && Robot.isTurnComplete) newState(MainState.state1Diagonal);
						break;
					
					
					case state1Diagonal:
						Robot.strafe(8, 135, -45, 1, .2, 0);
						if (Robot.currentInches > 1) Wobble.armDown();
						if (Robot.isStrafeComplete) newState(MainState.state2WobbleGoal);
						break;
					
					
					//put down wobble goal and back up
					case state2WobbleGoal:
						Wobble.armDown();
						if (mainTime.seconds() > .9) {
							newState(MainState.state3Turn);
							Wobble.armFold();
							Wobble.gripperOpen();
						}
						break;
					
					
					//turn to power shot shooting position
					case state3Turn:
						Robot.turn(-127, .8, .4);
						
						if (Robot.isTurnComplete) {
							newState(MainState.state4Drive);
							Wobble.gripperHalf();
						}
						break;
					
					
					//drive to power shot shooting position
					case state4Drive:
						Robot.strafe(22.5, -126, -126, 1, 0, 0);
						if (Robot.isStrafeComplete) newState(MainState.state5Turn);
						break;
					
					
					//turn towards power shots
					case state5Turn:
						if (mainTime.seconds() > .8) {
							Robot.turn(-4, 1, 0);
							Shooter.powerShot();
						}
						
						if (mainTime.seconds() > .8 && Robot.isTurnComplete) newState(MainState.state6PS1);
						break;
					
					
					case state6PS1:
						Shooter.powerShot();
						
						if (Shooter.feederCount() < 1) {
							Robot.setPowerAuto(0, 0, 2);
							if (mainTime.seconds() > .8) Shooter.feederState(true, 1.5);
						} else {
							Robot.turn(7, 1, 1);
							if (mainTime.seconds() > 1.7 && Robot.isTurnComplete) newState(MainState.state7PS2);
						}
						break;
					
					
					//shoot power shot 2, turn to power shot 3
					case state7PS2:
						Shooter.powerShot();
						if (Shooter.feederCount() < 2) {
							Robot.setPowerAuto(0, 0, 7);
							if (mainTime.seconds() > .2) Shooter.feederState(true, 1.5);
						} else {
							Robot.turn(13, 1, 1);
							if (Robot.isTurnComplete) newState(MainState.state8PS3);
						}
						break;
					
					
					//shoot power shot 3, turn to second wobble goal
					case state8PS3:
						Shooter.powerShot();
						Wobble.armDown();
						Wobble.gripperOpen();
						
						if (Shooter.feederCount() < 3) {
							if (mainTime.seconds() > 0) Shooter.feederState(true, 1.5);
							Robot.setPowerAuto(0, 0, 13);
						} else {
							Robot.turn(-37, 1, .8);
							Shooter.shooterOff();
							if (Robot.isTurnComplete) newState(MainState.state9Drive);
						}
						break;
					
					//drive to second wobblee goal
					case state9Drive:
						Robot.strafe(18, -37, 143, 1, 0, 0);
						if (Robot.isStrafeComplete) {
							newState(MainState.state10WobbleGoal);
							Robot.setPower(0, 0, 0, 1);
						}
						break;
					
					
					//pick up second wobble goal
					case state10WobbleGoal:
						Wobble.armDown();
						if (mainTime.seconds() > 1) Wobble.gripperGrip();
						
						if (mainTime.seconds() > 1.6) {
							Wobble.armTele();
							Robot.strafe(7, -37, 143, 1, 0, 0);
							
							if (Robot.isStrafeComplete) {
								newState(MainState.state11Turn);
								Robot.setPower(0, 0, 0, 1);
							}
						}
						break;
					
					
					//turn to face back wall
					case state11Turn:
						Robot.turn(0, 1, 1);
						Intake.fabricGroundRings();
						
						if (Robot.isTurnComplete) {
							newState(MainState.state12Drive);
							Robot.setPower(0, 0, 0, 1);
						}
						break;
					
					
					//strafe to line up with ring stack
					case state12Drive:
						Robot.strafe(6, 0, 90, 1, 0, 0);
						
						if (Robot.isStrafeComplete) {
							Robot.setPower(0, 0, 0, 1);
							newState(MainState.state13Drive);
							break;
						}
						break;
					
					
					//quickly push over stack and separate the rings
					case state13Drive:
						Robot.strafe(24, 0, 0, .5, .2, .3);
						Intake.intakeStallControl();
						if (mainTime.seconds() > .7 && Robot.isStrafeComplete) {
							Shooter.shooterPID.resetIntegralSum();
							Robot.setPower(0, 0, 0, 1);
							newState(MainState.state14Drive);
							break;
						}
						break;
					
					
					//slowly advance forward with intake to get 3 rings
					case state14Drive:
						Robot.strafe(8, 0, 0, .3, .3, 0);
						Intake.intakeStallControl();
						if (Robot.isStrafeComplete) newState(MainState.state15Shoot);
						break;
					
					
					//shoot first three rings
					case state15Shoot:
						Robot.setPowerAuto(0, 0, 0);
						Shooter.setRPM(3100);
						
						if (mainTime.seconds() > .8) {
							Intake.intakeOff();
							Intake.fabricRetract();
						}
						
						if (mainTime.seconds() > 1) Shooter.feederState(true, 1.5);
						if (mainTime.seconds() > 1.6 && Shooter.feederCount() > 4) newState(MainState.state18Turn);
						break;
					
					//turn towards target B
					case state18Turn:
						Robot.turn(180, 1, 0);
						Shooter.shooterOff();
						Wobble.armPosition(.3);
						
						if (mainTime.seconds() > .5 && Robot.isTurnComplete) {
							newState(MainState.state19Drive);
							Robot.setPower(0, 0, 0, 1);
							break;
						}
						break;
					
					
					//drive to target B and open the gripper
					case state19Drive:
						Robot.strafe(14, 180, 0, .6, .3, 0);
						
						if (Robot.isStrafeComplete) {
							Robot.setPower(0, 0, 0, 1);
							newState(MainState.state20WobbleGoal);
						}
						break;
					
					case state20WobbleGoal:
						Wobble.armDown();
						Intake.fabricGroundRings();
						Robot.setPowerAuto(0,0, Robot.closestTarget(180), 1);
						if(mainTime.seconds() > .8){
						
						}
						break;
					
				}
				
				break;
			
			case 4:
				switch (currentMainState) {
					//drive to target C
					case state1Drive:
						Robot.strafe(74, -90, 90, 1, .3, 0);
						
						if (Robot.currentInches > 60) Wobble.armDown();
						
						if (Robot.isStrafeComplete) {
							Robot.setPower(0, 0, 0, 1);
							newState(MainState.state3Turn);
						}
						break;
					
					
					//turn to power shot shooting position
					case state3Turn:
						if (mainTime.seconds() > .8) Robot.turn(-47, 1, .7);
						if (mainTime.seconds() > .9 && Robot.isTurnComplete) newState(MainState.state4Drive);
						break;
					
					
					//drive to power shot shooting position
					case state4Drive:
						Wobble.gripperOpen();
						
						if (mainTime.seconds() > 1){
							Wobble.armFold();
							Wobble.gripperHalf();
						}
						Robot.strafe(49, -49, -49, 1, .3, 0);
						
						if (Robot.isStrafeComplete) newState(MainState.state5Turn);
						break;
					
					
					//turn towards power shots
					case state5Turn:
						if (mainTime.seconds() > .6) {
							Robot.turn(90, 1, 1);
							Shooter.powerShot();
							Intake.fabricRollingRings();
						}
						if (Sensors.frontCamera.isTowerFound() && Sensors.gyro.angleRange(70, 110)) {
							newState(MainState.state6PS1);
							break;
						}
						break;
					
					
					//shoot power shot 1
					case state6PS1:
						Shooter.powerShot();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.centerPSAimAngle() - .4);
						Shooter.feederState(mainTime.seconds() > .6 &&
								                    Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60), 1.2);
						if (mainTime.seconds() > .7 && Shooter.feederCount() > 0) newState(MainState.state7PS2);
						break;
					
					
					//shoot power shot 2
					case state7PS2:
						Shooter.powerShot();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.rightPSAimAngle() - 1.5);
						Shooter.feederState(mainTime.seconds() > .5 &&
								                    Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60), 1.2);
						if (mainTime.seconds() > .6 && Shooter.feederCount() > 1) newState(MainState.state8PS3);
						break;
					
					
					//shoot power shot 3
					case state8PS3:
						Shooter.powerShot();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.leftPSAimAngle() - .5);
						Shooter.feederState(mainTime.seconds() > .5 &&
								                    Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60), 1.6);
						if (mainTime.seconds() > .6 && Shooter.feederCount() > 2) newState(MainState.state9Drive);
						break;
						
					
					
					//drive to second wobble goal
					case state9Drive:
						Robot.strafe(21, 50, 230, 1, .3, 0);
						Shooter.shooterOff();
						Wobble.armDown();
						Wobble.gripperOpen();
						Shooter.setFeederCount(0);
						if (Robot.isStrafeComplete && mainTime.seconds() > .4) {
							newState(MainState.state10WobbleGoal);
							Robot.setPower(0, 0, 0, 1);
						}
						break;
					
						
					//grip wobble goal and line up to tower
					case state10WobbleGoal:
						Wobble.armDown();
						if (mainTime.seconds() > .5) Wobble.gripperGrip();
						if (mainTime.seconds() > 1.1) {
							Wobble.armTele();
							Robot.strafe(20, 90, -167, 1, .3, 0);
							if(Robot.currentInches > 14) {  Shooter.highTower(true); }
							if (Robot.isStrafeComplete) newState(MainState.state11Drive);
						}
						break;
						
					
					//forward to ring stack
					case state11Drive:
						Intake.setFabricPosition(.09);
						Robot.strafe(8, 90, 90, .5, .2, .15);
						Shooter.highTower(true);
						if (Robot.isStrafeComplete) newState(MainState.state12Drive);
						break;
					
						
					//drive forward while intaking and shooting into high tower
					case state12Drive:
						Robot.strafe(20, Sensors.gyro.rawAngle() + Sensors.frontCamera.towerAimError() - 3, 90, .18, .15, .2);
						Intake.setFabricPosition(.26);
						Intake.setPower(.65);
						Shooter.highTower(true);
						Shooter.feederState(Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60), 1.5);
						if (Robot.isStrafeComplete) newState(MainState.state13Drive);
						break;
					
					
					//drive to target c for second wobble goal
					case state13Drive:
						Intake.intakeOff(); Intake.fabricRetract();
						Shooter.shooterOff(); Shooter.resetFeeder(); Shooter.lockFeeder();
						Wobble.armPosition(.15);
						Robot.strafe(73.5, 0, 100, 1, .15, 0);
						if(mainTime.seconds() > .4 && Robot.isStrafeComplete) newState(MainState.state20Drive);
						break;
						
						
					//drive out of square
					case state20Drive:
						Intake.intakeStallControl(); Intake.fabricGroundRings();
						Wobble.armDown();
						Wobble.gripperOpen();
						if(mainTime.seconds() > .2) Robot.strafe(39, 0, 10, .7, .3, 0);
						if(mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(MainState.state21Drive);
						break;
					
						
					//strafe to launch line
					case state21Drive:
						Robot.strafe(85, 0, -100, 1, 1, 0);
						Wobble.armFold();
						Wobble.gripperHalf();
						Intake.intakeStallControl();
						Intake.fabricGroundRings();
						Shooter.setFeederCount(0);
						if(Robot.isStrafeComplete) newState(MainState.state22Turn);
						break;
					
					case state22Turn:
						Robot.setPowerAuto(0, 0, 110);
						if(Sensors.gyro.angleRange(100, 120)) newState(MainState.state22Shoot);
						break;
						
					case state22Shoot:
						Robot.setPowerAuto(0, 0, Sensors.gyro.rawAngle() + Sensors.frontCamera.towerAimError() - 3);
						Shooter.highTower(true);
						Shooter.feederState(Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60), 1.5);
						Intake.intakeOff();
						Intake.fabricRollingRings();
						if(Shooter.feederCount() > 2) newState(MainState.state23Drive);
						break;
						
					case state23Drive:
						Robot.strafe(6, 90, 90, 1, .5, 0);
						Shooter.shooterOff(); Shooter.resetFeeder(); Shooter.lockFeeder();
						if(Robot.isStrafeComplete) newState(MainState.stateFinished);
						break;
						
					case stateFinished:
						Robot.setPowerVision(0,0,90);
						break;
				}
				break;
		}
		
		loopTelemetry();
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
		state9Turn,
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
		state1Turn,
		state22Shoot,
		state22Turn,
		state23Drive
	}
	
	private void loopTelemetry(){
		telemetry.addData("current state: ", currentMainState);
		telemetry.addData("drive", Robot.drive);
		telemetry.addData("strafe", Robot.strafe);
		telemetry.addData("turn", Robot.turn);
		telemetry.addData("power", Robot.power);
		telemetry.addData("current angle", Sensors.gyro.modAngle());
		telemetry.update();
	}
}
