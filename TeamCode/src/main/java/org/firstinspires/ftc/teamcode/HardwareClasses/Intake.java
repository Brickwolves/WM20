package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.utilities.Utils.hardwareMap;

import org.firstinspires.ftc.utilities.RingBufferOwen;

public class Intake {
    
    private static DcMotor intakeDrive;
    private static Servo bumperLeft, bumperRight;
    
    private final static double RETRACTED = 0.38, ZERO_RING = 0.015, ONE_RING = 0.11, TWO_RING = 0.03, THREE_RING = 0.14, FOUR_RING = 0.25;
    private final static double SERVO_DIFF = .09;
    
    private final static double INTAKE_ON = .87, INTAKE_REVERSE = .75;
    
    private final static double TICKS_PER_ROTATION = 28;
    private static double intakeRPM;
    
    public static double bumperPosition;
    
    private static final ElapsedTime stallTime = new ElapsedTime();
    public static ElapsedTime bumperTime = new ElapsedTime();
    
    static RingBufferOwen positionRing = new RingBufferOwen(5);
    static RingBufferOwen timeRing = new RingBufferOwen(5);
    
    public static IntakeState currentIntakeState = IntakeState.OFF;
    private static StallState currentStallState = StallState.START;
    public static BumperState currentBumperState = BumperState.RETRACT;
    
    
    public static void init(){
        intakeDrive = hardwareMap().get(DcMotor.class, "intake");
        bumperLeft = hardwareMap().get(Servo.class, "bumperleft");
        bumperRight = hardwareMap().get(Servo.class, "bumperright");
        
        intakeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        bumperRight.setDirection(Servo.Direction.REVERSE);
    }
    
    
    public static void setBumperPosition(double position){ bumperPosition = position; bumperLeft.setPosition(position); bumperRight.setPosition(position - SERVO_DIFF); }
    
    public static void retractBumper(){ bumperPosition = RETRACTED; setBumperPosition(RETRACTED); }
    
    public static void setBumperThreshold(int ringThreshold){
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
    
    public static void bumperState(boolean deployToggle, boolean rollingRings){
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
    
    public static double updateRPM(){
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
    
    public static void intakeOn(){ intakeDrive.setPower(INTAKE_ON); }
    
    public static void intakeStallControl(){
        switch(currentStallState){
            case START:
                if(stallTime.seconds() > .5) newState(StallState.ON);
                intakeOn();
                break;
            
            case ON:
                if(intakeDrive.getPower() < .2 && intakeDrive.getPower() > -.2) { newState(StallState.START); break; }
                if(updateRPM() < 300 && stallTime.seconds() > .2) newState(StallState.REVERSE);
                intakeOn();
                break;
                
            case REVERSE:
                if(intakeDrive.getPower() < .2 && intakeDrive.getPower() > -.2) { newState(StallState.START); break; }
                if(stallTime.seconds() > .2) newState(StallState.ON);
                intakeReverse();
                break;
        }
    }
    
    public static void intakeOff(){ intakeDrive.setPower(0.0); }
    
    public static void intakeReverse(){ intakeDrive.setPower(-INTAKE_REVERSE); }
    
    public static void intakeState(boolean intakeOnOff, boolean intakeReverse, boolean intakeHoldOn){
        switch (currentIntakeState) {
            
            case OFF:
                if (intakeOnOff) {
                    newState(IntakeState.ON);
                    if(currentBumperState == BumperState.RETRACT) newState(BumperState.DEPLOY);
                    if(Shooter.currentShooterState != Shooter.ShooterState.OFF) Shooter.newState(Shooter.ShooterState.OFF);
                    break;
                }
                
                if(intakeHoldOn) { intakeStallControl(); if(currentBumperState == BumperState.RETRACT) newState(BumperState.DEPLOY); }
                else if(intakeReverse) { intakeReverse(); if(currentBumperState == BumperState.RETRACT) newState(BumperState.DEPLOY); }
                else intakeOff();
                break;
            
                
            case ON:
                if (intakeReverse) { intakeReverse(); break; }
                if (intakeOnOff) { newState(IntakeState.OFF); break; }
                intakeStallControl();
                break;
        }
    }
    
    
    
    
    public static void newState(IntakeState newState) {
        stallTime.reset();
        currentIntakeState = newState;
    }
    
    private static void newState(StallState newState) {
        stallTime.reset();
        currentStallState = newState;
    }
    
    public static void newState(BumperState newState) {
        bumperTime.reset();
        currentBumperState = newState;
    }
    
    public enum IntakeState {
        OFF,
        ON,
    }
    
    private enum StallState {
        ON,
        REVERSE,
        START
    }
    
    public enum BumperState {
        RETRACT,
        DEPLOY,
        ROLLING
    }
    
}
