package org.firstinspires.ftc.teamcode.HardwareClasses;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.utilities.PID;
import org.firstinspires.ftc.utilities.RingBufferOwen;

public class Shooter {

    private final DcMotor shooterOne;
    private final DcMotor shooterTwo;
    private final Servo feeder;
    private final Servo feederLock;
    public PID shooterPID = new PID(.00025, 0.000002, 0.00009, 0.3, 40);

    private static final double TICKS_PER_ROTATION = 28;
    private static final double RING_FEED = 0.04;
    
    private static final double RESET = 0.31;
    private static final double FEEDER_LOCK = .46;
    private static final double FEEDER_UNLOCK = 0.2;
    
    private static final int TOP_GOAL = 3400;
    private static final int POWER_SHOT = 3050;
    
    private static final double FEED_TIME = .1;
    private static final double RESET_TIME = .09;
    private static final double LOCK_TIME = .8;
    private static final double UNLOCK_TIME = .06;
    
    private boolean isFeederLocked;
    private double shooterRPM;
    private static int feedCount = 0;

    RingBufferOwen timeRing = new RingBufferOwen(5);
    RingBufferOwen positionRing = new RingBufferOwen(5);
    
    public static FeederState currentFeederState = FeederState.STATE_IDLE;
    public static ShooterState currentShooterState = ShooterState.STATE_OFF;
    
    public ElapsedTime feederTime = new ElapsedTime();
    
    public Shooter(DcMotor shooterOne, DcMotor shooterTwo, Servo feeder, Servo feederLock) {
    
        shooterOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
        this.feeder = feeder;
        this.feederLock = feederLock;
    }

    public void feedRing(){
        feeder.setPosition(RING_FEED);
    }

    public void resetFeeder(){
        feeder.setPosition(RESET);
    }
    
    public void lockFeeder(){
        feederLock.setPosition(FEEDER_LOCK);
    }
    
    public void unlockFeeder(){
        feederLock.setPosition(FEEDER_UNLOCK);
    }
    
    public static double feederCount(){ return feedCount; }
    
    public void feederState(boolean trigger){
        switch (currentFeederState) {
            
            case STATE_IDLE:
                if(trigger && getPower() > .1 && currentShooterState != ShooterState.STATE_OFF){ newState(FeederState.STATE_FEED); }
                
                if(feederTime.seconds() > LOCK_TIME){ lockFeeder(); }
                else{ unlockFeeder(); }
                
                isFeederLocked = false;
                resetFeeder();
                break;
            
            case STATE_FEED:
                if (isFeederLocked) {
                    if (feederTime.seconds() > FEED_TIME + UNLOCK_TIME) { newState(FeederState.STATE_RESET); feedCount++; }
                    if (feederTime.seconds() > UNLOCK_TIME) { feedRing(); }
                }else{
                    if (feederTime.seconds() > FEED_TIME) { newState(FeederState.STATE_RESET); feedCount++; }
                    feedRing();
                }
                unlockFeeder();
                break;
            
            case STATE_RESET:
                if (feederTime.seconds() > RESET_TIME) { newState(FeederState.STATE_IDLE); break; }
                resetFeeder();
                unlockFeeder();
                break;
        }
    }
    

    public void setPower(double power){
        shooterOne.setPower(power);
        shooterTwo.setPower(power);
    }
    
    public double getPower(){
        return (shooterOne.getPower() + shooterTwo.getPower()) / 2;
    }

    public void resetEncoders(){
        shooterOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public long getPosition(){
        return (shooterOne.getCurrentPosition() + shooterTwo.getCurrentPosition()) /  2;
    }

    public double updateRPM(){

        long currentTime = System.currentTimeMillis();
        long deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaMinutes = deltaMili / 60000.0;

        long currentPosition = getPosition();
        long deltaTicks = currentPosition - positionRing.getValue(currentPosition);
        double deltaRotations = deltaTicks / TICKS_PER_ROTATION;

        shooterRPM = Math.abs(deltaRotations / deltaMinutes);

        return shooterRPM;
    }
    
    public double getRPM(){
        return shooterRPM;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void setRPM(int targetRPM){
        shooterPID.setFComponent(targetRPM / 10000.0);
        
        double shooterPower = shooterPID.update(targetRPM - updateRPM());
    
        shooterPower = Range.clip(shooterPower,0.0, 1.0);
        setPower(shooterPower);
    }
    

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void highTower(){ setRPM(TOP_GOAL); }
    
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void highTower(double distance){ setRPM((int) (TOP_GOAL + (distance  - 70) * 10)); }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void powerShot(){ setRPM(POWER_SHOT); }

    public void shooterOff(){ setPower(0.0); }
    
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void shooterState(boolean shooterOnOff, boolean powerShot, boolean topGoal, double towerDistance){
        switch (currentShooterState) {
            
            case STATE_OFF:
                if (shooterOnOff || topGoal) {
                    newState(ShooterState.STATE_TOP_GOAL);
                    shooterPID.resetIntegralSum();
                    if(Intake.currentIntakeState != Intake.IntakeState.STATE_OFF){
                        Intake.newState(Intake.IntakeState.STATE_OFF);
                    }
                    break;
                }
                
                if (powerShot) {
                    newState(ShooterState.STATE_POWER_SHOT);
                    shooterPID.resetIntegralSum();
                    if(Intake.currentIntakeState != Intake.IntakeState.STATE_OFF){
                        Intake.newState(Intake.IntakeState.STATE_OFF);
                    }
                    break;
                }
                feedCount = 0;
                shooterOff();
                break;
                
            case STATE_TOP_GOAL:
                if (powerShot) { newState(ShooterState.STATE_POWER_SHOT);  break; }
                if (shooterOnOff || topGoal) { newState(ShooterState.STATE_OFF); break; }
                highTower();
                break;
                
            case STATE_POWER_SHOT:
                if (topGoal) { newState(ShooterState.STATE_TOP_GOAL);  break; }
                if (shooterOnOff || powerShot) { newState(ShooterState.STATE_OFF); break; }
                powerShot();
                break;
        }
    }
    
    
    private void newState(FeederState newState) { currentFeederState = newState; feederTime.reset(); }
    
    public static void newState(ShooterState newState) { currentShooterState = newState; }
    
    private enum FeederState {
        STATE_IDLE,
        STATE_RESET,
        STATE_FEED
    }
    
    public enum ShooterState {
        STATE_OFF,
        STATE_TOP_GOAL,
        STATE_POWER_SHOT
    }
    

}