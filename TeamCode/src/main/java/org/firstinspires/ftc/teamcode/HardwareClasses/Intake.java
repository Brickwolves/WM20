package org.firstinspires.ftc.teamcode.HardwareClasses;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.utilities.Utils.hardwareMap;

import org.firstinspires.ftc.utilities.PID;
import org.firstinspires.ftc.utilities.RingBufferOwen;

public class Intake {
    
    private static DcMotor intakeDriveOne, intakeDriveTwo;
    private static Servo fabricLeft, fabricRight;
    
    public static PID intakePID = new PID(.0012, 0.000, 0.000, 0, 50);
    
    private final static double RETRACTED = 0.37, ROLLING_RINGS = 0.21, GROUND_RINGS = 0.14;
    private final static double SERVO_DIFF = .09;
    
    private final static int INTAKE_ON = 190;
    private final static double INTAKE_REVERSE = .45;
    
    private final static double TICKS_PER_ROTATION = 384.5;
    private static double intakeRPM;
    public static double targetRPM;
    
    public static double fabricPosition;
    
    private static final ElapsedTime stallTime = new ElapsedTime();
    public static ElapsedTime bumperTime = new ElapsedTime();
    
    static RingBufferOwen positionRing = new RingBufferOwen(3);
    static RingBufferOwen timeRing = new RingBufferOwen(3);
    
    public static IntakeState currentIntakeState;
    private static StallState currentStallState;
    public static FabricState currentFabricState;
    
    
    public static void init(){
        intakeDriveOne = hardwareMap().get(DcMotor.class, "intakeone");
        intakeDriveTwo = hardwareMap().get(DcMotor.class, "intaketwo");
        fabricLeft = hardwareMap().get(Servo.class, "bumperleft");
        fabricRight = hardwareMap().get(Servo.class, "bumperright");
        
        intakeDriveOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeDriveOne.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeDriveTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeDriveTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        fabricRight.setDirection(Servo.Direction.REVERSE);
    
        currentIntakeState = IntakeState.OFF;
        currentStallState = StallState.START;
        currentFabricState = FabricState.GROUND;
    }
    
    
    public static void setFabricPosition(double position){ fabricPosition = position; fabricLeft.setPosition(position); fabricRight.setPosition(position - SERVO_DIFF); }
    
    public static void fabricRetract(){ fabricPosition = RETRACTED; setFabricPosition(RETRACTED); }
    
    public static void fabricRollingRings(){ fabricPosition = ROLLING_RINGS; setFabricPosition(ROLLING_RINGS); }
    
    public static void fabricGroundRings(){ fabricPosition = GROUND_RINGS; setFabricPosition(GROUND_RINGS); }
    
    
    public static void fabricState(boolean deployToggle, boolean groundRings){
        switch (currentFabricState) {
            
            case RETRACT:
                if (deployToggle) { newState(FabricState.ROLLING); break; }
                if (groundRings) { newState(FabricState.GROUND); break; }
                fabricRetract();
                break;
            
            case ROLLING:
                if (deployToggle) { newState(FabricState.RETRACT); newState(IntakeState.OFF); break; }
                if (groundRings) { newState(FabricState.GROUND); break; }
                fabricRollingRings();
                break;
    
            case GROUND:
                if (!groundRings) { newState(FabricState.ROLLING); break; }
                fabricGroundRings();
                break;
        }
    }
    
    public static double updateRPM(){
        long currentTime = Sensors.currentTimeMillis();
        long deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaMinutes = deltaMili / 60000.0;
    
        long currentPosition = intakeDriveOne.getCurrentPosition();
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
    
    public static void setPower(double power){ intakeDriveOne.setPower(power); intakeDriveTwo.setPower(power); }
    
    public static double getPower(){ return (intakeDriveOne.getPower() + intakeDriveTwo.getPower()) / 2; }
    
    public static void intakeOn(){ setPower(.37); }
    
    public static void intakeStallControl(int targetRPM){
        switch(currentStallState){
            case START:
                if(stallTime.seconds() > .5) newState(StallState.ON);
                intakeOn();
                break;
            
            case ON:
                if(intakeDriveOne.getPower() < .1 && intakeDriveOne.getPower() > -.1) { newState(StallState.START); break; }
                if(updateRPM() < 55 && stallTime.seconds() > .3) newState(StallState.REVERSE);
                intakeOn();
                break;
                
            case REVERSE:
                if(intakeDriveOne.getPower() < .1 && intakeDriveOne.getPower() > -.1) { newState(StallState.START); break; }
                if(stallTime.seconds() > .4) newState(StallState.ON);
                intakeReverse();
                break;
        }
    }
    
    public static void intakeStallControl(){
        intakeStallControl(INTAKE_ON);
    }
    
    public static void intakeOff(){ intakeDriveOne.setPower(0.0); intakeDriveTwo.setPower(0.0); targetRPM = 0; }
    
    public static void intakeReverse(){ intakeDriveOne.setPower(-INTAKE_REVERSE); intakeDriveTwo.setPower(-INTAKE_REVERSE); }
    
    public static void intakeState(boolean intakeOnOff, boolean intakeReverse, boolean intakeHoldOn){
        switch (currentIntakeState) {
            
            case OFF:
                if (intakeOnOff) {
                    newState(IntakeState.ON);
                    if(currentFabricState == FabricState.RETRACT) newState(FabricState.GROUND);
                    break;
                }
                
                if(intakeHoldOn) { intakeStallControl(); if(currentFabricState == FabricState.RETRACT) newState(FabricState.GROUND); }
                else if(intakeReverse) { intakeReverse(); if(currentFabricState == FabricState.RETRACT) newState(FabricState.GROUND); }
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
    
    public static void newState(FabricState newState) {
        bumperTime.reset();
        currentFabricState = newState;
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
    
    public enum FabricState {
        RETRACT,
        GROUND,
        ROLLING
    }
    
}
