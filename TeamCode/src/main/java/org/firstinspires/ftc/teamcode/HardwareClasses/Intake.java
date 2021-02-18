package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.utilities.RingBufferOwen;

public class Intake {
    
    private DcMotor intakeDrive;
    private Servo reachOne;
    private Servo reachTwo;
    private final static double RETRACTED = 0.26;
    private final static double DEPLOYED = 0.0;
    private final static double INTAKE_ON = 1.0;
    private final static double INTAKE_REVERSE = .75;
    private final static double TICKS_PER_ROTATION = 28;
    private double intakeRPM;
    private ElapsedTime stallTime = new ElapsedTime();
    
    RingBufferOwen positionRing = new RingBufferOwen(15);
    RingBufferOwen timeRing = new RingBufferOwen(15);
    
    private IntakeState currentIntakeState = IntakeState.STATE_OFF;
    private IntakeState previousIntakeState = IntakeState.STATE_OFF;
    private StallState currentStallState = StallState.STATE_START;
    private ReachState currentReachState = ReachState.STATE_RETRACT;
    
    public Intake(DcMotor intakeDrive, Servo reachOne, Servo reachTwo){
        intakeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        reachTwo.setDirection(Servo.Direction.REVERSE);
        this.intakeDrive = intakeDrive;
        this.reachOne = reachOne;
        this.reachTwo = reachTwo;
    }
    
    public void retractReach(){
        reachOne.setPosition(RETRACTED);
        reachTwo.setPosition(RETRACTED+.03);
    }
    
    public void deployReach(){
        reachOne.setPosition(DEPLOYED);
        reachTwo.setPosition(DEPLOYED);
    }
    
    public void reachState(boolean deployToggle){
        switch (currentReachState) {
            
            case STATE_RETRACT:
                if (deployToggle) { newState(ReachState.STATE_DEPLOY); break; }
                retractReach();
                break;
            
            case STATE_DEPLOY:
                if (deployToggle) { newState(ReachState.STATE_RETRACT); newState(IntakeState.STATE_OFF); break; }
                deployReach();
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
                if(updateRPM() < 200 && stallTime.seconds() > .8){
                    newState(StallState.STATE_REVERSE);
                }
                break;
                
            case STATE_REVERSE:
                intakeReverse();
                if(stallTime.seconds() > .35){
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
