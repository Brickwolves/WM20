package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.utilities.Utils.hardwareMap;

public class Wobble {

    private static Servo gripperRight, gripperLeft;
    private static Servo lifter;
    
    private static final double GRIP = .653, OPEN = 0.23, HALF = 0.48;
    
    private static final double ARM_UP = .68, ARM_TELE = .84, ARM_DOWN = 0.1, ARM_FOLD = .99, SERVO_DIFF = .13;
    
    private static ArmState currentArmState;
    private static GripperState currentGripperState;
    private static final ElapsedTime gripperTime = new ElapsedTime();
    
    
    public static void init(){
        gripperRight = hardwareMap().get(Servo.class, "wobblegripperright");
        gripperLeft = hardwareMap().get(Servo.class, "wobblegripperleft");
        lifter = hardwareMap().get(Servo.class, "wobblearm");
        gripperLeft.setDirection(Servo.Direction.REVERSE);
    
        currentArmState = ArmState.FOLD;
        currentGripperState = GripperState.GRIP;
    }
    
    public static double gripperPosition() { return gripperRight.getPosition(); }
    
    public static void gripperGrip() { gripperRight.setPosition(GRIP); gripperLeft.setPosition(GRIP - SERVO_DIFF); }
    
    public static void gripperOpen() { gripperRight.setPosition(OPEN); gripperLeft.setPosition(OPEN - SERVO_DIFF); }
    
    public static void gripperHalf() { gripperRight.setPosition(HALF);gripperLeft.setPosition(HALF - SERVO_DIFF); }
    
    public static void gripperState(boolean openClose){
        switch (currentGripperState){
    
            case GRIP:
                if(openClose && currentArmState != ArmState.FOLD) newState(GripperState.OPEN);
                gripperGrip();
                break;
    
            case OPEN:
                if(openClose) newState(GripperState.GRIP);
                if(currentArmState == ArmState.FOLD) newState(GripperState.HALF);
                gripperOpen();
                break;
                
            case HALF:
                if(currentArmState != ArmState.FOLD) newState(GripperState.OPEN);
                gripperHalf();
                break;
        }
    }
    

    public static void armUp() { lifter.setPosition(ARM_UP); }

    public static void armDown() { lifter.setPosition(ARM_DOWN); }
    
    public static void armFold() { lifter.setPosition(ARM_FOLD); }
    
    public static void armTele() { lifter.setPosition(ARM_TELE);}
    
    public static void armPosition(double position) { lifter.setPosition(position);}
    

    public static void armState(boolean armUpDown, boolean armFold){
        switch(currentArmState){
                
            case UP:
                if(armUpDown) newState(ArmState.DOWN);
                if(armFold) newState(ArmState.FOLD);
                armUp();
                break;
    
            case DOWN:
                if(armUpDown) newState(ArmState.UP);
                if(armFold) newState(ArmState.FOLD);
                armDown();
                break;
    
            case FOLD:
                if(armUpDown && currentGripperState == GripperState.HALF) newState(ArmState.DOWN);
                else if(armUpDown) newState(ArmState.UP);
                if(armFold) newState(ArmState.DOWN);
                if(currentGripperState != GripperState.GRIP) armFold();
                else armTele();
                break;
        }
    }
    
    public static void newState(GripperState newState) {
        currentGripperState = newState;
        gripperTime.reset();
    }
    
    public enum GripperState {
        GRIP,
        OPEN,
        HALF
    }
    
    public static void newState(ArmState newState) { currentArmState = newState; }
    
    public enum ArmState {
        STATE_CONTROL,
        UP,
        DOWN,
        FOLD
    }
}
