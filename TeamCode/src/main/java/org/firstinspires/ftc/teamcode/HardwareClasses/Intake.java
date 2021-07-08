package org.firstinspires.ftc.teamcode.HardwareClasses;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.utilities.Utils.hardwareMap;

import org.firstinspires.ftc.utilities.PID;
import org.firstinspires.ftc.utilities.RingBufferOwen;

public class Intake {
    
    public static DcMotor intakeDriveFront, intakeDriveBack;
    private static Servo bumperLeft, bumperRight;
    private static CRServo bottomRoller;
    
    
    public static PID intakePID = new PID(.0012, 0.000, 0.000, 0, 50);
    
    private final static double RETRACTED = 0.32, ROLLING_RINGS = .16, GROUND_RINGS = 0.04;
    private final static double SERVO_DIFF = .025;
    
    private final static double INTAKE_ON = .7;
    private final static double INTAKE_REVERSE = .7;
    
    private final static double TICKS_PER_ROTATION = 384.5;
    private static double intakeRPM;
    public static double targetRPM;
    
    public static double bumperPosition;
    
    private static final ElapsedTime stallTime = new ElapsedTime();
    public static ElapsedTime bumperTime = new ElapsedTime();
    
    static RingBufferOwen positionRing = new RingBufferOwen(3);
    static RingBufferOwen timeRing = new RingBufferOwen(3);
    
    public static IntakeState currentIntakeState;
    private static StallState currentStallState;
    public static BumperState currentBumperState;
    
    
    public static void init(){
        intakeDriveFront = hardwareMap().get(DcMotor.class, "intakefront");
        intakeDriveBack = hardwareMap().get(DcMotor.class, "intakeback");
        bumperLeft = hardwareMap().get(Servo.class, "bumperleft");
        bumperRight = hardwareMap().get(Servo.class, "bumperright");
        bottomRoller = hardwareMap().get(CRServo.class, "bottomroller");
        
        intakeDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDriveFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeDriveBack.setDirection(DcMotorSimple.Direction.REVERSE);
        bumperRight.setDirection(Servo.Direction.REVERSE);
        bottomRoller.setDirection(DcMotorSimple.Direction.REVERSE);
    
        currentIntakeState = IntakeState.OFF;
        currentStallState = StallState.START;
        currentBumperState = BumperState.GROUND;
    }
    
    
    public static void setBumperPosition(double position){ bumperPosition = position; bumperLeft.setPosition(position); bumperRight.setPosition(position - SERVO_DIFF); }
    
    public static void bumperRetract(){ bumperPosition = RETRACTED; setBumperPosition(RETRACTED); }
    
    public static void bumperRollingRings(){ bumperPosition = ROLLING_RINGS; setBumperPosition(ROLLING_RINGS); }
    
    public static void bumperGroundRings(){ bumperPosition = GROUND_RINGS; setBumperPosition(GROUND_RINGS); }
    
    
    public static void bumperState(boolean deployToggle, boolean rollingRings){
        switch (currentBumperState) {
            
            case RETRACT:
                if (deployToggle) { newState(BumperState.GROUND); break; }
                if (rollingRings) { newState(BumperState.ROLLING); break; }
                bumperRetract();
                break;
            
            case GROUND:
                if (deployToggle) { newState(BumperState.RETRACT); newState(IntakeState.OFF); break; }
                if (rollingRings) { newState(BumperState.ROLLING); break; }
                bumperGroundRings();
                break;
    
            case ROLLING:
                if (!rollingRings) { newState(BumperState.GROUND); break; }
                bumperRollingRings();
                break;
        }
    }
    
    public static double updateRPM(){
        long currentTime = Sensors.currentTimeMillis();
        long deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaMinutes = deltaMili / 60000.0;
    
        long currentPosition = intakeDriveFront.getCurrentPosition();
        long deltaTicks = currentPosition - positionRing.getValue(currentPosition);
        double deltaRotations = deltaTicks / TICKS_PER_ROTATION;
    
        intakeRPM = Math.abs(deltaRotations / deltaMinutes);
    
        return intakeRPM;
    }
    
    public static double getRPM(){
        return intakeRPM;
    }
    
    @RequiresApi(api = Build.VERSION_CODES.N)
    public static void setRPM(int targetRPM){
        Intake.targetRPM = targetRPM;
        
        intakePID.setFComponent(targetRPM / 360.0);
        
        double intakePower = intakePID.update(targetRPM - updateRPM());
        
        if(getRPM() < targetRPM * .9){
            intakePID.setIntegralSum(targetRPM * .8);
        }
        
        intakePower = Range.clip(intakePower,0.0, 1.0);
        setPower(intakePower);
    }
    
    public static void setPower(double power){ intakeDriveFront.setPower(power); intakeDriveBack.setPower(power); }
    
    public static double getPower(){ return (intakeDriveFront.getPower() + intakeDriveBack.getPower()) / 2; }
    
    
    public static void intakeOn(){ setPower(INTAKE_ON); bottomRoller.setPower(1); }
    
    public static void intakeOff(){ intakeDriveFront.setPower(0.0); intakeDriveBack.setPower(0.0); bottomRoller.setPower(0); targetRPM = 0; }
    
    public static void intakeReverse(){ intakeDriveFront.setPower(-INTAKE_REVERSE); intakeDriveBack.setPower(-INTAKE_REVERSE); bottomRoller.setPower(0); }
    
    
    
    public static void intakeStallControl(){
        switch(currentStallState){
            case START:
                if(stallTime.seconds() > .5) { newState(StallState.ON); }
                intakeOn();
                break;
            
            case ON:
                if(intakeDriveFront.getPower() < .1 && intakeDriveFront.getPower() > -.1) { newState(StallState.START); break; }
                if(updateRPM() < 55 && stallTime.seconds() > .3) newState(StallState.REVERSE);
                intakeOn();
                break;
            
            case REVERSE:
                if(intakeDriveFront.getPower() < .1 && intakeDriveFront.getPower() > -.1) { newState(StallState.START); break; }
                if(stallTime.seconds() > .4) newState(StallState.ON);
                intakeReverse();
                break;
        }
    }
    
    public static void intakeState(boolean intakeOnOff, boolean intakeReverse, boolean intakeHoldOn){
        switch (currentIntakeState) {
            
            case OFF:
                if (intakeOnOff) {
                    newState(IntakeState.ON);
                    Shooter.newState(Shooter.ShooterState.OFF);
                    if(currentBumperState == BumperState.RETRACT) newState(BumperState.GROUND);
                    break;
                }
                
                if(intakeHoldOn) { intakeStallControl(); if(currentBumperState == BumperState.RETRACT) newState(BumperState.GROUND); }
                else if(intakeReverse)  intakeReverse();
                else intakeOff();
                break;
            
                
            case ON:
                if (intakeReverse) { intakeReverse(); break; }
                if (intakeOnOff) { newState(IntakeState.OFF); break; }
                intakeOn();
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
        OFF, ON
    }
    
    private enum StallState {
        ON, REVERSE, START
    }
    
    public enum BumperState {
        RETRACT, GROUND, ROLLING
    }
    
}
