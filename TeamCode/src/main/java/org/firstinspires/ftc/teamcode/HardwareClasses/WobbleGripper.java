package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.utilities.RingBuffer;

public class WobbleGripper {

    public Servo gripperOne, gripperTwo, lifter;
    
    private RingBuffer<Double> timeRing = new RingBuffer<>(20, 0.0);
    
    private double armPosition;
    private static final double GRIP = .53, OPEN = 0.07, HALF = 0.35;
    
    private static final double ARM_UP = .68, ARM_TELE = .84, ARM_DOWN = 0.05, ARM_FOLD = 1;
    private static final double ARM_CONTROL_RATE = -.00005;
    
    private static ArmState currentArmState = ArmState.UP;
    private GripperState currentGripperState = GripperState.GRIP;
    private ElapsedTime gripperTime = new ElapsedTime();

    public WobbleGripper(Servo gripperOne, Servo gripperTwo, Servo lifter){
        gripperTwo.setDirection(Servo.Direction.REVERSE);
        
        this.gripperOne = gripperOne;
        this.gripperTwo = gripperTwo;
        this.lifter = lifter;
    }
    
    
    public void gripperGrip() { gripperOne.setPosition(GRIP); gripperTwo.setPosition(GRIP+.02); }
    
    public void gripperOpen() { gripperOne.setPosition(OPEN); gripperTwo.setPosition(OPEN+.02); }
    
    public void gripperHalf() { gripperOne.setPosition(HALF); gripperTwo.setPosition(HALF+.02); }
    
    public void gripperState(boolean openClose){
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
    

    public void armControl(double deltaPosition){
        
        double currentTime = System.currentTimeMillis();
        double deltaMili = currentTime - timeRing.getValue(currentTime);
        armPosition = lifter.getPosition() + deltaPosition * deltaMili * ARM_CONTROL_RATE;
        
        armPosition = Range.clip(armPosition, ARM_DOWN, ARM_FOLD);
        
        lifter.setPosition(armPosition);
    }

    public void armUp() { lifter.setPosition(ARM_UP); }

    public void armDown() { lifter.setPosition(ARM_DOWN); }
    
    public void armFold() { lifter.setPosition(ARM_FOLD); }
    
    public void armTele() { lifter.setPosition(ARM_TELE);}
    
    public void armPosition(double position) { lifter.setPosition(position);}
    

    public void armState(boolean armUpDown, boolean armFold){
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
                if(currentGripperState != GripperState.GRIP){ armFold(); }
                else{ armTele(); }
                break;
        }
    }
    
    public void newState(GripperState newState) {
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
