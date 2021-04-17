package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.utilities.RingBufferOwen;

public class Intake {
    
    private DcMotor intakeDrive;
    public Servo bumperLeft, bumperRight;
    
    private final static double RETRACTED = 0.38, ZERO_RING = 0.015, ONE_RING = 0.11, TWO_RING = 0.03, THREE_RING = 0.14, FOUR_RING = 0.25;
    private final static double SERVO_DIFF = .09;
    
    private final static double INTAKE_ON = .87, INTAKE_REVERSE = .75;
    
    private final static double TICKS_PER_ROTATION = 28;
    private double intakeRPM;
    
    public static double bumperPosition;
    
    private static ElapsedTime stallTime = new ElapsedTime();
    public static ElapsedTime bumperTime = new ElapsedTime();
    
    RingBufferOwen positionRing = new RingBufferOwen(5);
    RingBufferOwen timeRing = new RingBufferOwen(5);
    
    public static IntakeState currentIntakeState = IntakeState.OFF;
    public static IntakeState previousIntakeState = IntakeState.OFF;
    private StallState currentStallState = StallState.START;
    public static BumperState currentBumperState = BumperState.RETRACT;
    
    
    public Intake(DcMotor intakeDrive, Servo bumperLeft, Servo bumperRight){
        intakeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        
        bumperRight.setDirection(Servo.Direction.REVERSE);
        this.intakeDrive = intakeDrive;
        this.bumperLeft = bumperLeft;
        this.bumperRight = bumperRight;
    }
    
    public void setBumperPosition(double position){ bumperPosition = position; bumperLeft.setPosition(position); bumperRight.setPosition(position - SERVO_DIFF); }
    
    public void retractBumper(){ bumperPosition = RETRACTED; setBumperPosition(RETRACTED); }
    
    public void setBumperThreshold(int ringThreshold){
        switch (ringThreshold){
            case 0:
                setBumperPosition(ZERO_RING);
                break;
    
            case 1:
                setBumperPosition(ONE_RING);
                break;
    
            case 2:
                setBumperPosition(TWO_RING);
                break;
    
            case 3:
                setBumperPosition(THREE_RING);
                break;
    
            case 4:
                setBumperPosition(FOUR_RING);
                break;
        }
        
    }
    
    public void bumperState(boolean deployToggle, boolean rollingRings){
        switch (currentBumperState) {
            
            case RETRACT:
                if (deployToggle) { newState(BumperState.DEPLOY); break; }
                if (rollingRings) { newState(BumperState.ROLLING); break; }
                retractBumper();
                break;
            
            case DEPLOY:
                if (deployToggle) { newState(BumperState.RETRACT); newState(IntakeState.OFF); break; }
                if (rollingRings) { newState(BumperState.ROLLING); break; }
                setBumperThreshold(1);
                break;
    
            case ROLLING:
                if (!rollingRings) { newState(BumperState.DEPLOY); break; }
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
            case START:
                intakeOn();
                newState(StallState.ON);
                break;
                
            case ON:
                intakeOn();
                if(updateRPM() < 300 && stallTime.seconds() > .2){
                    newState(StallState.REVERSE);
                }
                break;
                
            case REVERSE:
                intakeReverse();
                if(stallTime.seconds() > .2){
                    newState(StallState.ON);
                }
                break;
        }
    }
    
    public void intakeOff(){ intakeDrive.setPower(0.0); }
    
    public void intakeReverse(){ intakeDrive.setPower(-INTAKE_REVERSE); }
    
    public void intakeState(boolean intakeOnOff, boolean intakeReverse, boolean intakeHoldOn){
        switch (currentIntakeState) {
            
            case OFF:
                if (intakeOnOff) {
                    newState(IntakeState.ON);
                    if(currentBumperState == BumperState.RETRACT) { newState(BumperState.DEPLOY); }
                    if(Shooter.currentShooterState != Shooter.ShooterState.OFF){
                        Shooter.newState(Shooter.ShooterState.OFF);
                    }
                    break;
                }
                if (intakeReverse) {
                    newState(IntakeState.REVERSE);
                    if(currentBumperState == BumperState.RETRACT) { newState(BumperState.DEPLOY); }
                    break; }
                if(intakeHoldOn) intakeStallControl();
                else intakeOff();
                break;
            
                
            case ON:
                if (intakeReverse) { newState(IntakeState.REVERSE); break; }
                if (intakeOnOff) { newState(IntakeState.OFF); break; }
                intakeStallControl();
                break;
                
            case REVERSE:
                if(!intakeReverse){
                    switch(previousIntakeState){
                        case ON: newState(IntakeState.ON); break;
                        case OFF: newState(IntakeState.OFF); break;
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
        OFF,
        ON,
        REVERSE
    }
    
    private enum StallState {
        ON,
        REVERSE,
        START
    }
    
    public static enum BumperState {
        RETRACT,
        DEPLOY,
        ROLLING
    }
    
}
