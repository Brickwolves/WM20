package org.firstinspires.ftc.teamcode.HardwareClasses;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.utilities.MathUtils;
import org.firstinspires.ftc.utilities.PID;
import org.firstinspires.ftc.utilities.RingBuffer;
import org.firstinspires.ftc.utilities.RingBufferOwen;

import static org.firstinspires.ftc.utilities.MathUtils.degCos;
import static org.firstinspires.ftc.utilities.MathUtils.degTan;
import static org.firstinspires.ftc.utilities.Utils.hardwareMap;

public class Shooter {

    private static DcMotor shooterOne, shooterTwo;
    private static Servo feeder, feederLock, turret;
    public static PID shooterPID = new PID(.00024, 0.00003, 0.00012, 0.3, 50);

    private static final double TICKS_PER_ROTATION = 28;
    
    private static final double RING_FEED = .34, RESET = .6;
    private static final double FEEDER_LOCK = .46, FEEDER_UNLOCK = .2;
    
    private static final double FEED_TIME = .1, RESET_TIME = .14, PS_DELAY = .4;
    private static final double LOCK_TIME = .8, UNLOCK_TIME = .08;
    
    private static final double TURRET_SERVO_R = .935, TURRET_SERVO_L = .45, TURRET_SERVO_RANGE = TURRET_SERVO_R - TURRET_SERVO_L;
    private static final double TURRET_ANGLE_R = -22.5, TURRET_ANGLE_L = 37, TURRET_ANGLE_RANGE = TURRET_ANGLE_R - TURRET_ANGLE_L;
    
    private static final int TOP_GOAL = 3550, POWER_SHOT = 2900;
    
    private static boolean isFeederLocked;
    private static double shooterRPM;
    private static int feedCount = 0;
    public static boolean shooterJustOn = false, feederJustOn = false;
    public static double targetRPM;

    static RingBufferOwen timeRing = new RingBufferOwen(3);
    static RingBufferOwen positionRing = new RingBufferOwen(3);
    static RingBuffer<Double> shortAngleRing = new RingBuffer<>(2,0.0);
    static RingBuffer<Long> shortTimeRing = new RingBuffer<>(2, (long)0);
    public static FeederState currentFeederState;
    public static ShooterState currentShooterState;
    
    public static ElapsedTime feederTime = new ElapsedTime();
    public static ElapsedTime shooterTime = new ElapsedTime();
    
    
    
    public static void init() {
        shooterOne = hardwareMap().get(DcMotor.class, "shooterone");
        shooterTwo = hardwareMap().get(DcMotor.class, "shootertwo");
        feeder = hardwareMap().get(Servo.class, "feeder");
        feederLock = hardwareMap().get(Servo.class, "feederlock");
        turret = hardwareMap().get(Servo.class, "telescope");
        
        shooterOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        currentFeederState = FeederState.IDLE;
        currentShooterState = ShooterState.OFF;
    }
    
    
    public static void setTurretAngle(double turretAngle){
        
        long deltaMiliShort = Sensors.currentTimeMilis() - shortTimeRing.getValue(Sensors.currentTimeMilis());
        double deltaSecondsShort = deltaMiliShort / 1000.0;
    
        double deltaAngleShort = turretAngle - shortAngleRing.getValue(turretAngle);
        double rateOfChangeShort = deltaAngleShort/deltaSecondsShort;
        
        turretAngle += rateOfChangeShort * .17;
        
        turretAngle = Range.clip(turretAngle, TURRET_ANGLE_R, TURRET_ANGLE_L);
        
        double servoPos = (((turretAngle - TURRET_ANGLE_R) * TURRET_SERVO_RANGE) / TURRET_ANGLE_RANGE) + TURRET_SERVO_R;
        
        turret.setPosition(servoPos);
    }
    
    public static double getTurretAngle(){
        return (((turret.getPosition() - TURRET_SERVO_R) * TURRET_ANGLE_RANGE) / TURRET_SERVO_RANGE) + TURRET_ANGLE_R;
    }
    
    public static double verticalComponent(){
        double xComponent = MathUtils.degSin(getTurretAngle());
        double yComponent = Math.sqrt(.2061 - .2061 * Math.pow(xComponent, 2));
        return ((MathUtils.degASin(yComponent) - 27) * .9) + 27;
    }
    
    public static void turretAim(){ turretAim(true); }
    
    public static void turretAim(boolean autoAim){
        if(Sensors.frontCamera.isTowerFound() && autoAim && Sensors.gyro.angleRange(30, 150) && getPower() > .1)
            setTurretAngle(Sensors.frontCamera.towerAimError() + .5);
        else setTurretAngle(0);
    }
    
    
    
    public static void feedRing(){ feeder.setPosition(RING_FEED); }
    
    public static void resetFeeder(){ feeder.setPosition(RESET); }
    
    public static void lockFeeder(){ feederLock.setPosition(FEEDER_LOCK); }
    
    public static void unlockFeeder(){ feederLock.setPosition(FEEDER_UNLOCK); }
    
    public static void setFeederCount(int feederCount){ feedCount = feederCount; }
    
    public static double feederCount(){ return feedCount; }
    
    
    public static void feederAutoState(int feedCount){
        feederState(Shooter.feedCount <= feedCount);
    }
    
    public static void feederState(boolean trigger){
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
                    if (feederTime.seconds() > FEED_TIME + UNLOCK_TIME) { newState(FeederState.RESET);  }
                    if (feederTime.seconds() > UNLOCK_TIME) { feedRing(); }
                }else{
                    if (feederTime.seconds() > FEED_TIME) { newState(FeederState.RESET); }
                    feedRing();
                }
                unlockFeeder();
                break;
            
            case RESET:
                if (currentShooterState != ShooterState.POWER_SHOT && feederTime.seconds() > RESET_TIME) { newState(FeederState.IDLE); feedCount++; break; }
                if (currentShooterState == ShooterState.POWER_SHOT && feederTime.seconds() > RESET_TIME) { newState(FeederState.PS_DELAY); feederJustOn = true; feedCount++; break; }
                resetFeeder();
                unlockFeeder();
                break;
    
            case PS_DELAY:
                if (feederTime.seconds() > PS_DELAY) { newState(FeederState.IDLE); break; }
                resetFeeder();
                unlockFeeder();
                break;
        }
    }
    

    public static void setPower(double power){
        shooterOne.setPower(power);
        shooterTwo.setPower(power);
    }
    
    public static double getPower(){
        return (shooterOne.getPower() + shooterTwo.getPower()) / 2;
    }

    public static long getPosition(){
        return (shooterOne.getCurrentPosition() + shooterTwo.getCurrentPosition()) /  2;
    }

    public static double updateRPM(){

        long currentTime = System.currentTimeMillis();
        long deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaMinutes = deltaMili / 60000.0;

        long currentPosition = getPosition();
        long deltaTicks = currentPosition - positionRing.getValue(currentPosition);
        double deltaRotations = deltaTicks / TICKS_PER_ROTATION;

        shooterRPM = Math.abs(deltaRotations / deltaMinutes);

        return shooterRPM;
    }
    
    public static double getRPM(){
        return shooterRPM;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void setRPM(int targetRPM){
        Shooter.targetRPM = targetRPM;
        
        shooterPID.setFComponent(targetRPM / 10000.0);
        
        double shooterPower = shooterPID.update(targetRPM - updateRPM());
    
        if(getRPM() < targetRPM * .9){
            shooterPID.setIntegralSum(targetRPM * .8);
        }
        
        shooterPower = Range.clip(shooterPower,0.0, 1.0);
        setPower(shooterPower);
    }
    

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void highTower(boolean autoPower){
        double towerDistance = Sensors.frontCamera.towerDistance() / 100;
        int RPM;
        if(towerDistance < 1.8 || !Sensors.frontCamera.isTowerFound() || !autoPower){
             RPM = TOP_GOAL;
        }else {
            RPM = (int) (137 * (Math.sqrt(9.8 * Math.pow(towerDistance, 3.8) /
                                             (2 * degCos(verticalComponent()) * degCos(verticalComponent()) * (.9 * degTan(verticalComponent()) * towerDistance - .796)))));
        }
        
        setRPM(RPM);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void powerShot(){ setRPM(POWER_SHOT); }

    public static void shooterOff(){ setPower(0.0); }
    
    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void shooterState(boolean shooterOnOff, boolean powerShot, boolean topGoal, boolean visionAim){
        shooterJustOn = false;
        switch (currentShooterState) {
            
            case OFF:
                if (shooterOnOff || topGoal) {
                    newState(ShooterState.TOP_GOAL);
                    shooterPID.resetIntegralSum();
                    
                    shooterJustOn = true;
                    break;
                }
                
                if (powerShot) {
                    newState(ShooterState.POWER_SHOT);
                    shooterPID.resetIntegralSum();
                    shooterJustOn = true;
                    break;
                }
                feedCount = 0;
                shooterOff();
                break;
                
            case TOP_GOAL:
                if (powerShot) { newState(ShooterState.POWER_SHOT); shooterJustOn = true; break; }
                if (shooterOnOff || topGoal) { newState(ShooterState.OFF); break; }
                /*if(shooterTime.seconds() > .5 && shooterTime.seconds() < .58 && Intake.currentIntakeState != Intake.IntakeState.OFF){
                    Intake.newState(Intake.IntakeState.OFF);
                }*/
                //telescopeAim(visionAim);
                highTower(visionAim);
                break;
                
            case POWER_SHOT:
                if (topGoal) { newState(ShooterState.TOP_GOAL);  shooterJustOn = true; break; }
                if (shooterOnOff || powerShot) { newState(ShooterState.OFF); break; }
                if(shooterTime.seconds() > .5 && shooterTime.seconds() < .58 && Intake.currentIntakeState != Intake.IntakeState.OFF){
                    Intake.newState(Intake.IntakeState.OFF);
                }
                powerShot();
                break;
        }
    }
    
    
    private static void newState(FeederState newState) { currentFeederState = newState; feederTime.reset(); }
    
    public static void newState(ShooterState newState) { currentShooterState = newState; shooterTime.reset();}
    
    private enum FeederState {
        IDLE,
        RESET,
        FEED,
        PS_DELAY
    }
    
    public enum ShooterState {
        OFF,
        TOP_GOAL,
        POWER_SHOT
    }
    

}