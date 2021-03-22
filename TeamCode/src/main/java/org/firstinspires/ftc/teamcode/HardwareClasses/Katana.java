package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareClasses.Shooter.ShooterState;

public class Katana {
	
	private static final double DOWN = 0.05;
	private static final double UP = 0.38;
	private static final double FULL_FOLD = 0.38;
	private static final double HALF_FOLD = 0.38;
	private static final double SHOOT = 0.38;
	
	Servo katanaLeft, katanaRight;
	
	public static KatanaState currentKatanaState = KatanaState.STATE_FOLD;
	
	public Katana(Servo katanaLeft, Servo katanaRight){
		this.katanaLeft = katanaLeft;
		this.katanaRight = katanaRight;
		this.katanaRight.setDirection(Servo.Direction.REVERSE);
	}
	
	public void katanaDown(){ katanaRight.setPosition(DOWN); katanaLeft.setPosition(DOWN); }
	
	public void katanaUp(){ katanaRight.setPosition(UP); katanaLeft.setPosition(UP); }
	
	public void katanaShoot(){ katanaRight.setPosition(SHOOT); katanaLeft.setPosition(SHOOT); }
	
	public void katanaFullFold(){ katanaRight.setPosition(FULL_FOLD); katanaLeft.setPosition(FULL_FOLD); }
	
	public void katanaHalfFold(){ katanaRight.setPosition(HALF_FOLD); katanaLeft.setPosition(HALF_FOLD); }
	
	
	
	
	
	
	public void katanaState(boolean foldToggle){
		
			if(foldToggle || (Gyro.getModAngle() > 115 || Gyro.getModAngle() < 65)){
				if(Shooter.currentShooterState != ShooterState.STATE_OFF){ katanaShoot(); }
				else if(Intake.currentBumperState == Intake.BumperState.STATE_RETRACT){ katanaHalfFold(); }
				else{ katanaFullFold(); }
				
			}else{
				switch(currentKatanaState){
					case STATE_DOWN:
						
						if(Intake.currentIntakeState == Intake.IntakeState.STATE_OFF && Shooter.feederCount() < 1){
							newState(KatanaState.STATE_UP);
						}
						katanaDown();
						break;
					
					
					case STATE_UP:
						if(){}
						
						katanaUp();
						break;
				}
			}
	}
	
	
	
	public static enum KatanaState{
		STATE_DOWN,
		STATE_UP,
		STATE_FOLD
	}
	
	public static void newState(KatanaState newKatanaState){
		currentKatanaState = newKatanaState;
	}
}
