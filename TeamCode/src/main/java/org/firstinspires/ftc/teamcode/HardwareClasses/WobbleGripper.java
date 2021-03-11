package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.utilities.RingBuffer;

public class WobbleGripper {

    public Servo gripperOne;
    public Servo gripperTwo;
    public Servo lifter;
    
    private RingBuffer<Double> timeRing = new RingBuffer<>(20, 0.0);
    
    private double armPosition;
    private static final double BAT = .58;
    private static final double GRIP = .53;
    private static final double OPEN = 0.07;
    private static final double HALF = 0.35;
    private static final double ARM_UP = .68;
    private static final double ARM_TELE = .84;
    private static final double ARM_DOWN = 0.05;
    private static final double ARM_FOLD = 1;
    private static final double ARM_CONTROL_RATE = -.00005;
    
    private ArmState currentArmState = ArmState.STATE_UP;
    private GripperState currentGripperState = GripperState.STATE_GRIP;
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
    
    public void gripperBAT() { gripperOne.setPosition(BAT); gripperTwo.setPosition(BAT + .02);}
    
    public void gripperState(boolean openClose){
        switch (currentGripperState){
    
            case STATE_GRIP:
                if(openClose && currentArmState != ArmState.STATE_FOLD) { newState(GripperState.STATE_OPEN); break; }
                gripperGrip();
                break;
    
            case STATE_OPEN:
                if(openClose) { newState(GripperState.STATE_GRIP); break; }
                if(currentArmState == ArmState.STATE_FOLD) { newState(GripperState.STATE_HALF); break; }
                gripperOpen();
                break;
                
            case STATE_HALF:
                if(currentArmState != ArmState.STATE_FOLD) { newState(GripperState.STATE_OPEN); break; }
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
                
            case STATE_UP:
                if(armUpDown) { newState(ArmState.STATE_DOWN); break; }
                if(armFold) { newState(ArmState.STATE_FOLD); break; }
                armUp();
                break;
    
            case STATE_DOWN:
                if(armUpDown) { newState(ArmState.STATE_UP); break; }
                if(armFold) { newState(ArmState.STATE_FOLD); break; }
                armDown();
                break;
    
            case STATE_FOLD:
                if(armUpDown) { newState(ArmState.STATE_UP); break; }
                if(armFold) { newState(ArmState.STATE_DOWN); break; }
                if(currentGripperState != GripperState.STATE_GRIP ){ armFold(); }
                else{ armTele(); }
                break;
        }
    }
    
    public void newState(GripperState newState) {
        currentGripperState = newState;
        gripperTime.reset();
    }
    
    public enum GripperState {
        STATE_GRIP,
        STATE_OPEN,
        STATE_HALF
    }
    
    public void newState(ArmState newState) { currentArmState = newState; }
    
    public enum ArmState {
        STATE_CONTROL,
        STATE_UP,
        STATE_DOWN,
        STATE_FOLD
    }
}
