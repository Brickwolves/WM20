package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter.ShooterState;
import static org.firstinspires.ftc.utilities.Utils.hardwareMap;

public class Katana {
	
	private static final double DOWN = 0, UP = 0.4, FULL_FOLD = 1, HALF_FOLD = .86, SHOOT = 0.7;
	
	private static Servo katanaRight, katanaLeft;
	
	public static KatanaState currentKatanaState;
	
	
	public static void init(){
		katanaRight = hardwareMap().get(Servo.class, "katanaright");
		katanaLeft = hardwareMap().get(Servo.class, "katanaleft");
		
		katanaLeft.scaleRange(.55, .95);
		katanaRight.scaleRange(.28, .68);
		katanaLeft.setDirection(Servo.Direction.REVERSE);
		
		currentKatanaState = KatanaState.DOWN;
	}
	
	
	public static void setKatanaPosition(double position){ katanaLeft.setPosition(position); katanaRight.setPosition(position); }
	
	public static void katanaDown(){ setKatanaPosition(DOWN); }
	
	public static void katanaUp(){ setKatanaPosition(UP); }
	
	public static void katanaShoot(){ setKatanaPosition(SHOOT); }
	
	public static void katanaFullFold(){ setKatanaPosition(FULL_FOLD); }
	
	public static void katanaHalfFold(){ setKatanaPosition(HALF_FOLD); }
	
	public static void katanaAutoState(){
		if (Shooter.targetRPM != 0) {
			katanaShoot();
		} else if (Intake.bumperPosition >= .3) {
			katanaHalfFold();
		} else {
			katanaFullFold();
		}
	}
	
	public static void katanaState(boolean foldToggle, boolean forceDown){
		if(forceDown) {
			katanaDown();
				
		}else if(foldToggle || !Sensors.gyro.angleRange(58, 122)) {
			if (Shooter.currentShooterState != ShooterState.OFF) {
				katanaShoot();
			} else if (Intake.currentBumperState == Intake.BumperState.RETRACT && Intake.bumperTime.seconds() > .2) {
				katanaHalfFold();
			} else if (Intake.bumperTime.seconds() > .15){
				katanaFullFold();
			}
			
		}else{
			switch(currentKatanaState){
				case DOWN:
					if(Shooter.currentShooterState != Shooter.ShooterState.OFF && Shooter.feederCount() < 1){
						newState(KatanaState.UP);
					}
					katanaDown();
					break;
					
				case UP:
					if(Shooter.currentShooterState == Shooter.ShooterState.OFF || Shooter.feederCount() > 0){
						newState(KatanaState.DOWN);
					}
					katanaUp();
					break;
			}
		}
	}
	
	
	public enum KatanaState{
		DOWN,
		UP
	}
	
	public static void newState(KatanaState newKatanaState){
		currentKatanaState = newKatanaState;
	}
}
