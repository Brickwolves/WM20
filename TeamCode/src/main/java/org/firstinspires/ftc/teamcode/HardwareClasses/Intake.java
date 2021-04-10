package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.utilities.RingBufferOwen;

public class Intake {
    
    private DcMotor intakeDrive;
    public Servo bumperLeft;
    public Servo bumperRight;
    private final static double RETRACTED = 0.38;
    private final static double ZERO_RING = 0.015;
    private final static double ONE_RING = 0.1;
    private final static double TWO_RING = 0.03;
    private final static double THREE_RING = 0.14;
    private final static double FOUR_RING = 0.25;
    private final static double SERVO_DIFF = .09;
    private final static double INTAKE_ON = .87;
    private final static double INTAKE_REVERSE = .75;
    private final static double TICKS_PER_ROTATION = 28;
    private double intakeRPM;
    private static ElapsedTime stallTime = new ElapsedTime();
    public static ElapsedTime bumperTime = new ElapsedTime();
    
    RingBufferOwen positionRing = new RingBufferOwen(5);
    RingBufferOwen timeRing = new RingBufferOwen(5);
    
    public static IntakeState currentIntakeState = IntakeState.STATE_OFF;
    public static IntakeState previousIntakeState = IntakeState.STATE_OFF;
    private StallState currentStallState = StallState.STATE_START;
    public static BumperState currentBumperState = BumperState.STATE_RETRACT;
    
    public Intake(DcMotor intakeDrive, Servo bumperLeft, Servo bumperRight){
        intakeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bumperRight.setDirection(Servo.Direction.REVERSE);
        this.intakeDrive = intakeDrive;
        this.bumperLeft = bumperLeft;
        this.bumperRight = bumperRight;
    }
    
    public void retractBumper(){
        bumperLeft.setPosition(RETRACTED);
        bumperRight.setPosition(RETRACTED - .06);
    }
    
    public void setBumperThreshold(int ringThreshold){
        switch (ringThreshold){
            case 0:
                bumperLeft.setPosition(ZERO_RING);
                bumperRight.setPosition(ZERO_RING - SERVO_DIFF);
                break;
    
            case 1:
                bumperLeft.setPosition(ONE_RING);
                bumperRight.setPosition(ONE_RING - SERVO_DIFF);
                break;
    
            case 2:
                bumperLeft.setPosition(TWO_RING);
                bumperRight.setPosition(TWO_RING - SERVO_DIFF);
                break;
    
            case 3:
                bumperLeft.setPosition(THREE_RING);
                bumperRight.setPosition(THREE_RING - SERVO_DIFF);
                break;
    
            case 4:
                bumperLeft.setPosition(FOUR_RING);
                bumperRight.setPosition(FOUR_RING - SERVO_DIFF);
                break;
        }
        
    }
    
    public void bumperState(boolean deployToggle, boolean rollingRings){
        switch (currentBumperState) {
            
            case STATE_RETRACT:
                if (deployToggle) { newState(BumperState.STATE_DEPLOY); break; }
                if (rollingRings) { newState(BumperState.STATE_ROLLING); break; }
                retractBumper();
                break;
            
            case STATE_DEPLOY:
                if (deployToggle) { newState(BumperState.STATE_RETRACT); newState(IntakeState.STATE_OFF); break; }
                if (rollingRings) { newState(BumperState.STATE_ROLLING); break; }
                setBumperThreshold(1);
                break;
    
            case STATE_ROLLING:
                if (!rollingRings) { newState(BumperState.STATE_DEPLOY); break; }
                setBumperThreshold(4);
                break;
        }
    }
    
    public double updateRPM(){
        long currentTime = System.currentTimeMillis();
        long deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaMinutes = deltaMili / 60000.0;
    
        long currentPosition = intakeDrive.getCurrentPosition();
        long deltaTicks = currentPosition - positionRing.getValue(currentPosition);
        double deltaRotations = deltaTicks / TICKS_PER_ROTATION;
    
        intakeRPM = Math.abs(deltaRotations / deltaMinutes);
    
        return intakeRPM;
    }
    
    public double getRPM(){
        return intakeRPM;
    }
    
    public void intakeOn(){ intakeDrive.setPower(INTAKE_ON); }
    
    public void intakeStallControl(){
        switch(currentStallState){
            case STATE_START:
                intakeOn();
                newState(StallState.STATE_ON);
                break;
                
            case STATE_ON:
                intakeOn();
                if(updateRPM() < 200 && stallTime.seconds() > .3){
                    newState(StallState.STATE_REVERSE);
                }
                break;
                
            case STATE_REVERSE:
                intakeReverse();
                if(stallTime.seconds() > .2){
                    newState(StallState.STATE_ON);
                }
                break;
        }
    }
    
    public void intakeOff(){ intakeDrive.setPower(0.0); }
    
    public void intakeReverse(){ intakeDrive.setPower(INTAKE_ON * -INTAKE_REVERSE); }
    
    public void intakeState(boolean intakeOnOff, boolean intakeReverse){
        switch (currentIntakeState) {
            
            case STATE_OFF:
                if (intakeOnOff) {
                    newState(IntakeState.STATE_ON);
                    if(currentBumperState == BumperState.STATE_RETRACT) { newState(BumperState.STATE_DEPLOY); }
                    if(Shooter.currentShooterState != Shooter.ShooterState.STATE_OFF){
                        Shooter.newState(Shooter.ShooterState.STATE_OFF);
                    }
                    break;
                }
                
                if (intakeReverse) {
                    newState(IntakeState.STATE_REVERSE);
                    if(currentBumperState == BumperState.STATE_RETRACT) { newState(BumperState.STATE_DEPLOY); }
                    break; }
                intakeOff();
                break;
            
                
            case STATE_ON:
                if (intakeReverse) { newState(IntakeState.STATE_REVERSE); break; }
                if (intakeOnOff) { newState(IntakeState.STATE_OFF); break; }
                intakeStallControl();
                break;
                
            case STATE_REVERSE:
                if(!intakeReverse){
                    switch(previousIntakeState){
                        case STATE_ON: newState(IntakeState.STATE_ON); break;
                        case STATE_OFF: newState(IntakeState.STATE_OFF); break;
                    }
                }
                intakeReverse();
                break;
                
        }
    }
    
    
    
    
    public static void newState(IntakeState newState) {
        stallTime.reset();
        previousIntakeState = currentIntakeState;
        currentIntakeState = newState;
    }
    
    private void newState(StallState newState) {
        stallTime.reset();
        currentStallState = newState;
    }
    
    public static void newState(BumperState newState) {
        bumperTime.reset();
        currentBumperState = newState;
    }
    
    public static enum IntakeState {
        STATE_OFF,
        STATE_ON,
        STATE_REVERSE
    }
    
    private enum StallState {
        STATE_ON,
        STATE_REVERSE,
        STATE_START
    }
    
    public static enum BumperState {
        STATE_RETRACT,
        STATE_DEPLOY,
        STATE_ROLLING
    }
    
}
