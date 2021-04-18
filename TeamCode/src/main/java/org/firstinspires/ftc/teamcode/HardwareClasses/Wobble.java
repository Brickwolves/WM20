package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.utilities.RingBuffer;
import static org.firstinspires.ftc.utilities.Utils.getHardwareMap;

public class Wobble {

    private static Servo gripperOne, gripperTwo;
    public static Servo lifter;
    
    private static final double GRIP = .53, OPEN = 0.07, HALF = 0.35;
    
    private static final double ARM_UP = .68, ARM_TELE = .84, ARM_DOWN = 0.05, ARM_FOLD = 1;
    
    private static ArmState currentArmState = ArmState.UP;
    private static GripperState currentGripperState = GripperState.GRIP;
    private static final ElapsedTime gripperTime = new ElapsedTime();

    public Wobble(Servo gripperOne, Servo gripperTwo, Servo lifter){
        gripperTwo.setDirection(Servo.Direction.REVERSE);
        
        Wobble.gripperOne = gripperOne;
        Wobble.gripperTwo = gripperTwo;
        Wobble.lifter = lifter;
    }
    
    public static void init(){
        gripperOne = getHardwareMap().get(Servo.class, "gripperone");
        gripperTwo = getHardwareMap().get(Servo.class, "grippertwo");
        lifter = getHardwareMap().get(Servo.class, "lifter");
    }
    
    
    public static void gripperGrip() { gripperOne.setPosition(GRIP); gripperTwo.setPosition(GRIP+.02); }
    
    public static void gripperOpen() { gripperOne.setPosition(OPEN); gripperTwo.setPosition(OPEN+.02); }
    
    public static void gripperHalf() { gripperOne.setPosition(HALF); gripperTwo.setPosition(HALF+.02); }
    
    public static void gripperState(boolean openClose){
        switch (currentGripperState){
    
            case GRIP:
                if(openClose && currentArmState != ArmState.FOLD) { newState(GripperState.OPEN); break; }
                gripperGrip();
                break;
    
            case OPEN:
                if(openClose) { newState(GripperState.GRIP); break; }
                if(currentArmState == ArmState.FOLD) { newState(GripperState.HALF); break; }
                gripperOpen();
                break;
                
            case HALF:
                if(currentArmState != ArmState.FOLD) { newState(GripperState.OPEN); break; }
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
                if(armUpDown) { newState(ArmState.DOWN); break; }
                if(armFold) { newState(ArmState.FOLD); break; }
                armUp();
                break;
    
            case DOWN:
                if(armUpDown) { newState(ArmState.UP); break; }
                if(armFold) { newState(ArmState.FOLD); break; }
                armDown();
                break;
    
            case FOLD:
                if(armUpDown) { newState(ArmState.UP); break; }
                if(armFold) { newState(ArmState.DOWN); break; }
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
