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
    public PID shooterPID = new PID(.00018, 0.00003, 0.00012, 0.3, 50);

    private static final double TICKS_PER_ROTATION = 28;
    
    private static final double RING_FEED = .04, RESET = .31;
    private static final double FEEDER_LOCK = .46, FEEDER_UNLOCK = .2;
    
    private static final double FEED_TIME = .1, RESET_TIME = .09, PS_RESET_TIME = 1;
    private static final double LOCK_TIME = .8, UNLOCK_TIME = .06;
    
    private static final int TOP_GOAL = 3550, POWER_SHOT = 3050;
    
    private boolean isFeederLocked;
    private double shooterRPM;
    private static int feedCount = 0;
    public static boolean shooterJustOn = false, feederJustOn = false;
    public static double targetRPM;

    RingBufferOwen timeRing = new RingBufferOwen(3);
    RingBufferOwen positionRing = new RingBufferOwen(3);
    
    public static FeederState currentFeederState = FeederState.IDLE;
    public static ShooterState currentShooterState = ShooterState.OFF;
    
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
    
    public static void setFeederCount(int feederCount){ feedCount = feederCount; }
    
    public static double feederCount(){ return feedCount; }
    
    public void feederState(boolean trigger){
        feederJustOn = false;
        switch (currentFeederState) {
            
            case IDLE:
                if(trigger && getPower() > .1){ newState(FeederState.FEED); }
                
                if(feederTime.seconds() > LOCK_TIME){ lockFeeder(); }
                else{ unlockFeeder(); }
                
                isFeederLocked = false;
                resetFeeder();
                break;
            
            case FEED:
                if (isFeederLocked) {
                    if (feederTime.seconds() > FEED_TIME + UNLOCK_TIME) { newState(FeederState.RESET);feedCount++; }
                    if (feederTime.seconds() > UNLOCK_TIME) { feedRing(); }
                }else{
                    if (feederTime.seconds() > FEED_TIME) { newState(FeederState.RESET); feedCount++; }
                    feedRing();
                }
                unlockFeeder();
                break;
            
            case RESET:
                if (currentShooterState != ShooterState.POWER_SHOT && feederTime.seconds() > RESET_TIME) { newState(FeederState.IDLE);  break; }
                if (currentShooterState == ShooterState.POWER_SHOT && feederTime.seconds() > RESET_TIME) { newState(FeederState.IDLE);  feederJustOn = true;break; }
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
    
        if(getRPM() < targetRPM * .9){
            shooterPID.setIntegralSum(targetRPM * .8);
        }
        
        shooterPower = Range.clip(shooterPower,0.0, 1.0);
        setPower(shooterPower);
    }
    

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void highTower(boolean autoPower){
        double towerDistance = Sensors.frontCamera.towerDistance() / 100;
        int RPM;
        if(towerDistance < 1.8 || !Sensors.frontCamera.isTowerFound() || !autoPower){
             RPM = TOP_GOAL;
        }else {
            RPM = (int) ((137) * Math.sqrt((9.8 * Math.pow(towerDistance, 4)) / (.7426 * towerDistance - 1.264)));
        }
        targetRPM = RPM;
        setRPM(RPM);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void powerShot(){ setRPM(POWER_SHOT); }

    public void shooterOff(){ setPower(0.0); }
    
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void shooterState(boolean shooterOnOff, boolean powerShot, boolean topGoal, boolean autoPower){
        shooterJustOn = false;
        switch (currentShooterState) {
            
            case OFF:
                if (shooterOnOff || topGoal) {
                    newState(ShooterState.TOP_GOAL);
                    shooterPID.resetIntegralSum();
                    if(Intake.currentIntakeState != Intake.IntakeState.OFF){
                        Intake.newState(Intake.IntakeState.OFF);
                    }
                    shooterJustOn = true;
                    break;
                }
                
                if (powerShot) {
                    newState(ShooterState.POWER_SHOT);
                    shooterPID.resetIntegralSum();
                    if(Intake.currentIntakeState != Intake.IntakeState.OFF){
                        Intake.newState(Intake.IntakeState.OFF);
                    }
                    shooterJustOn = true;
                    break;
                }
                feedCount = 0;
                shooterOff();
                break;
                
            case TOP_GOAL:
                if (powerShot) { newState(ShooterState.POWER_SHOT); shooterJustOn = true; break; }
                if (shooterOnOff || topGoal) { newState(ShooterState.OFF); break; }
                highTower(autoPower);
                break;
                
            case POWER_SHOT:
                if (topGoal) { newState(ShooterState.TOP_GOAL);  shooterJustOn = true; break; }
                if (shooterOnOff || powerShot) { newState(ShooterState.OFF); break; }
                powerShot();
                break;
        }
    }
    
    
    private void newState(FeederState newState) { currentFeederState = newState; feederTime.reset(); }
    
    public static void newState(ShooterState newState) { currentShooterState = newState; }
    
    private enum FeederState {
        IDLE,
        RESET,
        FEED
    }
    
    public enum ShooterState {
        OFF,
        TOP_GOAL,
        POWER_SHOT
    }
    

}