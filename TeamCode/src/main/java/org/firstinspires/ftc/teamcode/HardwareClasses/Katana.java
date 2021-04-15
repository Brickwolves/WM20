package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter.ShooterState;

public class Katana {
	
	private static final double DOWN = 0, UP = 0.4, FULL_FOLD = 1, HALF_FOLD = .86, SHOOT = 0.7;
	
	Servo katanaRight, katanaLeft;
	
	public static KatanaState currentKatanaState = KatanaState.DOWN;
	
	public Katana(Servo katanaRight, Servo katanaLeft){
		katanaLeft.scaleRange(.55, .95);
		katanaRight.scaleRange(.28, .68);
		katanaLeft.setDirection(Servo.Direction.REVERSE);
		
		this.katanaRight = katanaRight;
		this.katanaLeft = katanaLeft;
	}
	
	public void setKatanaPosition(double position){ katanaLeft.setPosition(position); katanaRight.setPosition(position); }
	
	public void katanaDown(){ setKatanaPosition(DOWN); }
	
	public void katanaUp(){ setKatanaPosition(UP); }
	
	public void katanaShoot(){ setKatanaPosition(SHOOT); }
	
	public void katanaFullFold(){ setKatanaPosition(FULL_FOLD); }
	
	public void katanaHalfFold(){ setKatanaPosition(HALF_FOLD); }
	
	
	public void katanaState(boolean foldToggle, boolean forceDown){
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
					if(Intake.currentIntakeState == Intake.IntakeState.OFF && Shooter.feederCount() < 1){
						newState(KatanaState.UP);
					}
					katanaDown();
					break;
					
				case UP:
					if(Intake.currentIntakeState == Intake.IntakeState.ON || Shooter.feederCount() > 0){
						newState(KatanaState.DOWN);
					}
					katanaUp();
					break;
			}
		}
	}
	
	
	public static enum KatanaState{
		DOWN,
		UP
	}
	
	public static void newState(KatanaState newKatanaState){
		currentKatanaState = newKatanaState;
	}
}
