/*
package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClasses.BadController;
import org.firstinspires.ftc.teamcode.HardwareClasses.Intake;
import org.firstinspires.ftc.teamcode.HardwareClasses.Katana;
import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter;
import org.firstinspires.ftc.teamcode.HardwareClasses.Wobble;
import org.firstinspires.ftc.utilities.Utils;

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;

@Autonomous(name = "Clean Auto", group = "Auto")
@Disabled
public class CleanAutonomous extends OpMode {
	
	private BadController operator;
	
	
	private MainState currentMainState = MainState.state1Drive;
	private final ElapsedTime mainTime = new ElapsedTime();
	

	private static double ringCount = 0;
	private final boolean ringsFound = false;

	@Override
	public void init() {
		
		Utils.setHardwareMap(hardwareMap);
		Robot.init(); Sensors.init(); Shooter.init(); Intake.init(); Wobble.init(); Katana.init();
		
		operator = new BadController(gamepad2);
		Shooter.setTurretAngle(-27);
		
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
			Intake.bumperGroundRings();
		}else{
			Intake.intakeOff();
			Intake.bumperRetract();
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
		Shooter.setTurretAngle(0);
	}
	
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void loop() {
		Sensors.update(); operator.update();
		Katana.katanaAutoState();
		
		switch ((int) ringCount) {
			case 0:
				switch (currentMainState) {
					case state1Drive:
						Robot.strafe(14, -90, 80, 1, .3, 0);
						if(Robot.isStrafeComplete) newState(MainState.state2Turn);
						break;
					
					case state2Turn:
						if(mainTime.seconds() > .2) {
							Robot.turn(90, 1, .5);
							Wobble.armTele();
							if(Sensors.gyro.angleRange(70, 100)) newState(MainState.state4Drive);
						}
						break;
					
					case state4Drive:
						Robot.strafe(51, 90, 23, 1, .3, 0);
						Wobble.armTele();
						if(Robot.isStrafeComplete && mainTime.seconds() > .3) { newState(MainState.state6PS1); Shooter.setFeederCount(0); }
						break;
					
						
					//shoot power shot 1
					case state6PS1:
						Shooter.powerShot();
						Intake.bumperRollingRings();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.centerPSAimAngle() - .4);
						Shooter.feederState(mainTime.seconds() > .6 &&
								                    Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
						if (mainTime.seconds() > .7 && Shooter.feederCount() > 0) newState(MainState.state7PS2);
						break;
					
					
					//shoot power shot 2
					case state7PS2:
						Shooter.powerShot();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.rightPSAimAngle() - 1.5);
						Shooter.feederState(mainTime.seconds() > .5 &&
								                    Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
						if (mainTime.seconds() > .6 && Shooter.feederCount() > 1) newState(MainState.state8PS3);
						break;
					
					
					//shoot power shot 3
					case state8PS3:
						Shooter.powerShot();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.leftPSAimAngle() - .5);
						Shooter.feederState(mainTime.seconds() > .5 &&
								                    Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
						if (mainTime.seconds() > .6 && Shooter.feederCount() > 2) newState(MainState.state9Drive);
						break;
					
					
					case state9Drive:
						Robot.strafe(21, 90, 0, 1, .3, 0);
						Shooter.shooterOff();
						Intake.bumperGroundRings(); Intake.intakeStallControl();
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(MainState.state95Drive);
						break;
						
						
					case state95Drive:
						Robot.strafe(46.5, 90, 90, 1, .3, 0);
						Intake.bumperGroundRings(); Intake.intakeStallControl();
						if(mainTime.seconds() > .1 && Robot.isStrafeComplete) newState(MainState.state10Turn);
						break;
					
						
					case state10Turn:
						if(mainTime.seconds() > .7) {
							Robot.turn(180, 1, .5);
							Intake.bumperGroundRings();
							Intake.intakeStallControl();
							if (Sensors.gyro.angleRange(176, 184)) newState(MainState.state11Drive);
						}
						break;
					
					
					case state11Drive:
						Robot.strafe(10, 180, 90, .8, .3, 0);
						Intake.bumperGroundRings(); Intake.intakeStallControl();
						if(mainTime.seconds() > .2 && (mainTime.seconds() > 1.2 || Robot.isStrafeComplete)) newState(MainState.state12Drive);
						break;
					
					
					case state12Drive:
						Robot.strafe(41, 180, 180, 1, .3, 0);
						Intake.bumperGroundRings(); Intake.intakeStallControl();
						if(mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(MainState.state13Drive);
						break;
					
					
					case state13Drive:
						if(mainTime.seconds() > .7) Robot.strafe(68, 180, -90, 1, .3, 0);
						Intake.bumperGroundRings(); Intake.intakeStallControl();
						Wobble.armPosition(.16);
						if(mainTime.seconds() > .9 && Robot.isStrafeComplete) newState(MainState.state14Turn);
						break;
						
						
					case state14Turn:
						if(mainTime.seconds() > .1) Robot.turn(270, 1, .3);
						if(Sensors.gyro.angleRange(262, 280)) { Wobble.gripperOpen(); newState(MainState.state15Drive); }
						Intake.intakeStallControl();
						break;
						
					case state15Drive:
						Robot.strafe(8, 270, 270, .4, .3, 0);
						Wobble.armFold();
						Intake.intakeStallControl();
						if(mainTime.seconds() > .4) Wobble.gripperHalf();
						if(mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(MainState.state16Turn);
						break;
						
					case state16Turn:
						Robot.setPowerAuto(0, 0, Robot.closestTarget(10));
						Intake.intakeStallControl();
						if(Sensors.gyro.angleRange(340, 360)) newState(MainState.state165Turn);
						break;
						
					case state165Turn:
						Robot.turn(90, 1, 1);
						Shooter.highTower(false);
						Shooter.setFeederCount(0);
						Intake.intakeStallControl();
						if(Sensors.gyro.angleRange(75, 95)) newState(MainState.state17Shoot);
						break;
						
					case state17Shoot:
						Robot.setPowerVision(0, 0, Sensors.gyro.rawAngle() + Sensors.frontCamera.towerAimError());
						Shooter.highTower(true);
						Intake.intakeOff();
						Shooter.feederState(mainTime.seconds() > .7 && Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
						if(Shooter.feederCount() >= 4) newState(MainState.state18Drive);
						break;
					
						
					case state18Drive:
						Robot.strafe(18, 132, -58, 1, .3, 0);
						Wobble.armDown(); Wobble.gripperOpen();
						Shooter.shooterOff();
						if(mainTime.seconds() > .2 && Robot.isStrafeComplete) { newState(MainState.state19Wobble);  }
						break;
					
					
					case state19Wobble:
						if(mainTime.seconds() > .4) Wobble.gripperGrip();
						if(mainTime.seconds() > .8) newState(MainState.state20Turn);
						break;
					
					
					case state20Turn:
						Robot.turn(0, 1, .3);
						Wobble.armPosition(.17);
						if(mainTime.seconds() > .2 && Robot.isTurnComplete) newState(MainState.state21Drive);
						break;
						
						
					case state21Drive:
						Robot.strafe(56, 0, 94, 1, .3, 0);
						if(mainTime.seconds() > .2 && Robot.isStrafeComplete) { newState(MainState.stateFinished); }
						break;
						
						
					case stateFinished:
						if(mainTime.seconds() > .4 && mainTime.seconds() < .8) Wobble.gripperOpen();
						if(mainTime.seconds() > .7) Wobble.armFold();
						if(mainTime.seconds() > .9) Wobble.gripperGrip();
						Shooter.shooterOff();
						Robot.setPowerVision(0,0,Robot.closestTarget(0),1);
						break;
				}
				
				break;
			
			case 1:
				switch (currentMainState) {
					case state1Drive:
						Robot.strafe(14, -90, 80, 1, .3, 0);
						if(Robot.isStrafeComplete) newState(MainState.state2Turn);
						break;
						
					case state2Turn:
						if(mainTime.seconds() > .2) {
							Robot.turn(90, 1, .5);
							Shooter.highTower(true);
							Wobble.armTele();
							Intake.bumperRollingRings();
							if(Sensors.gyro.angleRange(70, 100)) newState(MainState.state3Shoot);
						}
						break;
						
					case state3Shoot:
						Robot.setPowerVision(0, 0, Sensors.gyro.rawAngle() + Sensors.frontCamera.towerAimError());
						Shooter.highTower(true);
						Shooter.feederState(abs(Sensors.frontCamera.towerAimError()) < 2 && Shooter.getRPM() > (Shooter.targetRPM - 40) && Shooter.getRPM() < (Shooter.targetRPM + 40));
						if(Shooter.feederCount() >= 1) newState(MainState.state4Drive);
						break;
						
					case state4Drive:
						if(mainTime.seconds() < .5) Shooter.highTower(true);
						else Shooter.shooterOff();
						Robot.strafe(51, 90, 23, 1, .3, 0);
						Intake.bumperGroundRings(); Intake.intakeOn();
						
						
						if(Robot.isStrafeComplete && mainTime.seconds() > .3) { newState(MainState.state6PS1); Shooter.setFeederCount(0); }
						break;
					
					//shoot power shot 1
					case state6PS1:
						Shooter.powerShot();
						Intake.intakeOff(); Intake.bumperRollingRings();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.centerPSAimAngle() - .4);
						Shooter.feederState(mainTime.seconds() > 1 &&
								                    Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
						if (mainTime.seconds() > .7 && Shooter.feederCount() > 0) newState(MainState.state7PS2);
						break;
					
					
					//shoot power shot 2
					case state7PS2:
						Shooter.powerShot();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.rightPSAimAngle() - 1);
						Shooter.feederState(mainTime.seconds() > .5 &&
								                    Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
						if (mainTime.seconds() > .6 && Shooter.feederCount() > 1) newState(MainState.state8PS3);
						break;
					
					
					//shoot power shot 3
					case state8PS3:
						Shooter.powerShot();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.leftPSAimAngle() - .5);
						Shooter.feederState(mainTime.seconds() > .5 &&
								                    Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
						if (mainTime.seconds() > .6 && Shooter.feederCount() > 2) newState(MainState.state9Drive);
						break;
						
					case state9Drive:
						Robot.strafe(62, 90, 50, 1, .3, 0);
						Shooter.shooterOff();
						Intake.bumperGroundRings(); Intake.intakeStallControl();
						if(Robot.isStrafeComplete) newState(MainState.state10Turn);
						break;
						
					case state10Turn:
						if(mainTime.seconds() > 1) {
							Robot.turn(180, 1, .5);
							Intake.bumperGroundRings();
							Intake.intakeStallControl();
							if (Sensors.gyro.angleRange(176, 184)) newState(MainState.state11Drive);
						}
						break;
						
						
					case state11Drive:
						Robot.strafe(12.5, 180, 90, .8, .3, 0);
						Intake.bumperGroundRings(); Intake.intakeStallControl();
						if(mainTime.seconds() > .2 && (mainTime.seconds() > 1 || Robot.isStrafeComplete)) newState(MainState.state12Drive);
						break;
					
					
					case state12Drive:
						Robot.strafe(36, 178.5, 178.5, 1, .3, 0);
						Intake.bumperGroundRings(); Intake.intakeStallControl();
						if(mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(MainState.state13Drive);
						break;
						
						
					case state13Drive:
						if(mainTime.seconds() > .8) Robot.strafe(80, 180, -90, 1, .3, 0);
						Intake.bumperGroundRings(); Intake.intakeStallControl();
						Wobble.armPosition(.16);
						if(mainTime.seconds() > .9 && Robot.currentInches > 23) Wobble.gripperOpen();
						if(mainTime.seconds() > .9 && Robot.isStrafeComplete) newState(MainState.state16Turn);
						break;
					
					
					case state16Turn:
						Robot.setPowerAuto(0, 0, Robot.closestTarget(335));
						Intake.intakeStallControl();
						if(Sensors.gyro.angleRange(300, 360)) newState(MainState.state165Turn);
						break;
					
						
					case state165Turn:
						Robot.turn(90, 1, 1);
						Shooter.highTower(false);
						Shooter.setFeederCount(0);
						Intake.intakeStallControl();
						if(Sensors.gyro.angleRange(50, 95)) newState(MainState.state17Shoot);
						break;
					
						
					case state17Shoot:
						Robot.setPowerVision(0, 0, Sensors.gyro.rawAngle() + Sensors.frontCamera.towerAimError());
						Shooter.highTower(true);
						Intake.intakeOff();
						Shooter.feederState(mainTime.seconds() > .7 && Shooter.getRPM() > (Shooter.targetRPM - 50) && Shooter.getRPM() < (Shooter.targetRPM + 50));
						if(Shooter.feederCount() >= 4) newState(MainState.state16Drive);
						break;
						
						
						
					case state16Drive:
						Robot.strafe(18.5, 123, -57, 1, .3, 0);
						Wobble.armDown(); Wobble.gripperOpen();
						Shooter.shooterOff();
						if(mainTime.seconds() > .2 && Robot.isStrafeComplete) { newState(MainState.state17Wobble);  }
						break;
						
						
					case state17Wobble:
						if(mainTime.seconds() > .4) Wobble.gripperGrip();
						if(mainTime.seconds() > .8) newState(MainState.state18Turn);
						break;
						
						
					case state18Turn:
						Robot.turn(-90, 1, .3);
						Wobble.armPosition(.17);
						if(Robot.isTurnComplete && mainTime.seconds() > .2) newState(MainState.state19Drive);
						break;
						
						
					case state19Drive:
						Robot.strafe(30, -90, 90, 1, .3, 0);
						if(mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(MainState.stateFinished);
						break;
						
						
					case stateFinished:
						if(mainTime.seconds() > .8) { Wobble.gripperOpen(); Robot.setPowerVision(0,0, Robot.closestTarget(-90),1); }
						if(mainTime.seconds() > 1.3)  Wobble.armFold();
						
						Shooter.shooterOff();
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
							Intake.bumperRollingRings();
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
								                    Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						if (mainTime.seconds() > .7 && Shooter.feederCount() > 0) newState(MainState.state7PS2);
						break;
					
					
					//shoot power shot 2
					case state7PS2:
						Shooter.powerShot();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.rightPSAimAngle() - 1.5);
						Shooter.feederState(mainTime.seconds() > .5 &&
								                    Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						if (mainTime.seconds() > .6 && Shooter.feederCount() > 1) newState(MainState.state8PS3);
						break;
					
					
					//shoot power shot 3
					case state8PS3:
						Shooter.powerShot();
						Robot.setPowerVision(0, 0, Sensors.frontCamera.leftPSAimAngle() - 1);
						Shooter.feederState(mainTime.seconds() > .5 &&
								                    Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						if (mainTime.seconds() > .6 && Shooter.feederCount() > 2) newState(MainState.state9Drive);
						break;
						
					
					
					//drive to second wobble goal
					case state9Drive:
						Robot.strafe(21, 50, 230, 1, .3, 0);
						Shooter.shooterOff(); Shooter.feederState(false);
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
							Robot.strafe(17, 90, -167, 1, .3, 0);
							if(Robot.currentInches > 14) {  Shooter.highTower(true); }
							if (Robot.isStrafeComplete) newState(MainState.state11Drive);
						}
						break;
						
					
					//forward to ring stack
					case state11Drive:
						Intake.setBumperPosition(.09);
						Robot.strafe(11, 90, 90, .6, .5, 0);
						Shooter.highTower(true);
						if (Robot.isStrafeComplete) newState(MainState.state115Drive);
						break;
						
						
					case state115Drive:
						Robot.strafe(4, 90, -90, .1, 1, 0);
						Intake.setBumperPosition(.21);
						if (Robot.isStrafeComplete) newState(MainState.state12Drive);
					
						
					//drive forward while intaking and shooting into high tower
					case state12Drive:
						Robot.strafe(13, Sensors.gyro.rawAngle() + Sensors.frontCamera.towerAimError() - 1, 90, .14, .15, .2);
						Intake.setBumperPosition(.21);
						Shooter.turretAim();
						Intake.setPower(.31);
						Shooter.highTower(true);
						Shooter.feederState(Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						if (Robot.isStrafeComplete) newState(MainState.state13Drive);
						break;
					
					
					//drive to target c for second wobble goal
					case state13Drive:
						Intake.bumperRetract();
						Intake.intakeOff();
						Shooter.setTurretAngle(0);
						Shooter.feederState(false);
						Shooter.shooterOff();
						Wobble.armPosition(.2);
						Robot.strafe(74, 0, 99, 1, .5, 0);
						if(mainTime.seconds() > .4 && Robot.isStrafeComplete) newState(MainState.state20Drive);
						break;
						
						
					//drive out of square
					case state20Drive:
						Intake.bumperGroundRings();
						Intake.intakeStallControl();
						Wobble.armDown();
						Wobble.gripperOpen();
						if(mainTime.seconds() > .2) Robot.strafe(39, 0, 10, 1, .5, 0);
						if(mainTime.seconds() > .2 && Robot.isStrafeComplete) newState(MainState.state21Drive);
						break;
					
						
					//strafe to launch line
					case state21Drive:
						Robot.strafe(90, 0, -100, 1, 1, 0);
						Wobble.armFold();
						Wobble.gripperHalf();
						Intake.intakeStallControl();
						Intake.setBumperPosition(.13);
						Shooter.setFeederCount(0);
						if(Robot.isStrafeComplete) newState(MainState.state22Turn);
						break;
					
					case state22Turn:
						Robot.setPowerAuto(0, 0, 110);
						Shooter.highTower(true);
						if(Sensors.gyro.angleRange(100, 120)) newState(MainState.state22Shoot);
						break;
						
					case state22Shoot:
						Robot.setPowerVision(0, 0, Sensors.gyro.rawAngle() + Sensors.frontCamera.towerAimError() - 1.2);
						Shooter.highTower(true);
						Shooter.feederState(Shooter.getRPM() > (Shooter.targetRPM - 60) && Shooter.getRPM() < (Shooter.targetRPM + 60));
						Intake.intakeOff();
						Intake.bumperRollingRings();
						if(Shooter.feederCount() > 2) newState(MainState.state23Drive);
						break;
						
					case state23Drive:
						Robot.strafe(6, 90, 90, 1, 1, 0);
						Shooter.shooterOff();
						Shooter.feederState(false);
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
		state2Turn,
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
		state3Shoot, state10Drive, state10Turn, state14Turn, state17Wobble, state15Turn, state16Turn, state19Wobble, state20Turn, state95Drive, state22Drive, state165Turn, state115Drive, state23Drive
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
*/
