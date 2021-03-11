package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.utilities.RingBufferOwen;

public class Intake {
    
    private DcMotor intakeDrive;
    public Servo bumperOne;
    public Servo bumperTwo;
    private final static double RETRACTED = 0.3;
    private final static double ZERO_RING = 0.005;
    private final static double ONE_RING = 0.031;
    private final static double TWO_RING = 0.03;
    private final static double THREE_RING = 0.062;
    private final static double FOUR_RING = 0.092;
    private final static double INTAKE_ON = .87;
    private final static double INTAKE_REVERSE = .75;
    private final static double TICKS_PER_ROTATION = 28;
    private double intakeRPM;
    private ElapsedTime stallTime = new ElapsedTime();
    
    RingBufferOwen positionRing = new RingBufferOwen(5);
    RingBufferOwen timeRing = new RingBufferOwen(5);
    
    private IntakeState currentIntakeState = IntakeState.STATE_OFF;
    private IntakeState previousIntakeState = IntakeState.STATE_OFF;
    private StallState currentStallState = StallState.STATE_START;
    private ReachState currentReachState = ReachState.STATE_RETRACT;
    
    public Intake(DcMotor intakeDrive, Servo bumperOne, Servo bumperTwo){
        intakeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bumperTwo.setDirection(Servo.Direction.REVERSE);
        this.intakeDrive = intakeDrive;
        this.bumperOne = bumperOne;
        this.bumperTwo = bumperTwo;
    }
    
    public void retractBumper(){
        bumperOne.setPosition(RETRACTED);
        bumperTwo.setPosition(RETRACTED + .05);
    }
    
    public void setBumperThreshold(int ringThreshold){
        switch (ringThreshold){
            case 0:
                bumperOne.setPosition(ZERO_RING);
                bumperTwo.setPosition(ZERO_RING + .03);
                break;
    
            case 1:
                bumperOne.setPosition(ONE_RING);
                bumperTwo.setPosition(ONE_RING + .03);
                break;
    
            case 2:
                bumperOne.setPosition(TWO_RING);
                bumperTwo.setPosition(TWO_RING + .03);
                break;
    
            case 3:
                bumperOne.setPosition(THREE_RING);
                bumperTwo.setPosition(THREE_RING + .03);
                break;
    
            case 4:
                bumperOne.setPosition(FOUR_RING);
                bumperTwo.setPosition(FOUR_RING + .05);
                break;
        }
        
    }
    
    public void reachState(boolean deployToggle){
        switch (currentReachState) {
            
            case STATE_RETRACT:
                if (deployToggle) { newState(ReachState.STATE_DEPLOY); break; }
                retractBumper();
                break;
            
            case STATE_DEPLOY:
                if (deployToggle) { newState(ReachState.STATE_RETRACT); newState(IntakeState.STATE_OFF); break; }
                setBumperThreshold(1);
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
    
    public void intakeState(boolean intakeOn, boolean intakeOff, boolean intakeReverse){
        switch (currentIntakeState) {
            
            case STATE_OFF:
                if (intakeOn) { newState(IntakeState.STATE_ON); newState(ReachState.STATE_DEPLOY); break; }
                if (intakeReverse) { newState(IntakeState.STATE_REVERSE); newState(ReachState.STATE_DEPLOY); break; }
                intakeOff();
                break;
            
            case STATE_ON:
                if (intakeReverse) { newState(IntakeState.STATE_REVERSE); break; }
                if (intakeOff) { newState(IntakeState.STATE_OFF); break; }
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
                
        }
    }
    
    
    
    
    private void newState(IntakeState newState) {
        stallTime.reset();
        previousIntakeState = currentIntakeState;
        currentIntakeState = newState;
    }
    
    private void newState(StallState newState) {
        stallTime.reset();
        currentStallState = newState;
    }
    
    private void newState(ReachState newState) {
        currentReachState = newState;
    }
    
    private enum IntakeState {
        STATE_OFF,
        STATE_ON,
        STATE_REVERSE
    }
    
    private enum StallState {
        STATE_ON,
        STATE_REVERSE,
        STATE_START
    }
    
    private enum ReachState {
        STATE_RETRACT,
        STATE_DEPLOY
    }
    
}
