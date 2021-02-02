package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.utilities.RingBuffer;

public class WobbleGripper {

    public CRServo gripperOne;
    public CRServo gripperTwo;
    public Servo lifter;
    
    private RingBuffer<Double> timeRing = new RingBuffer<>(20, 0.0);
    
    private double armPosition;
    private static final double WHEEL_GRIP = 1.0;
    private static final double WHEEL_EJECT = -1.0;
    private static final double ARM_UP = .4;
    private static final double ARM_DOWN = 0.8;
    private static final double ARM_FOLD = 0.0;
    private static final double ARM_CONTROL_RATE = -.00005;
    
    private ArmState currentArmState = ArmState.STATE_CONTROL;
    private GripperState currentGripperState = GripperState.STATE_OFF;

    public WobbleGripper(CRServo gripperOne, CRServo gripperTwo, Servo lifter){
        gripperTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        this.gripperOne = gripperOne;
        this.gripperTwo = gripperTwo;
        this.lifter = lifter;
    }
    
    
    
    public void wheelGrip() { gripperOne.setPower(WHEEL_GRIP); gripperTwo.setPower(WHEEL_GRIP); }
    
    public void wheelOff() { gripperOne.setPower(0.0); gripperTwo.setPower(0.0); }
    
    public void wheelEject() { gripperOne.setPower(WHEEL_EJECT); gripperTwo.setPower(WHEEL_EJECT); }
    
    public void gripperState(boolean onOff, boolean eject){
        switch (currentGripperState){
            
            case STATE_OFF:
                if(onOff) { newState(GripperState.STATE_GRIP); break; }
                if(eject) { newState(GripperState.STATE_EJECT); break; }
                wheelOff();
                break;
    
            case STATE_GRIP:
                if(onOff) { newState(GripperState.STATE_OFF); break; }
                if(eject) { newState(GripperState.STATE_EJECT); break; }
                wheelGrip();
                break;
    
            case STATE_EJECT:
                if(onOff) { newState(GripperState.STATE_OFF); break; }
                wheelEject();
                break;
        }
    }
    

    public void armControl(double deltaPosition){
        
        double currentTime = System.currentTimeMillis();
        double deltaMili = currentTime - timeRing.getValue(currentTime);
        armPosition = lifter.getPosition() + deltaPosition * deltaMili * ARM_CONTROL_RATE;
        
        armPosition = Range.clip(armPosition, ARM_FOLD, ARM_DOWN);
        
        lifter.setPosition(armPosition);
    }

    public void armUp() { lifter.setPosition(ARM_UP); }

    public void armDown() { lifter.setPosition(ARM_DOWN); }
    
    public void armFold() { lifter.setPosition(ARM_FOLD); }
    

    public void armState(double armControlUp, double armControlDown, boolean armUp, boolean armDown, boolean armFold){
        switch(currentArmState){
            
            case STATE_CONTROL:
                if(armUp) { newState(ArmState.STATE_UP); break; }
                if(armDown) { newState(ArmState.STATE_DOWN); break; }
                if(armFold) { newState(ArmState.STATE_FOLD); break; }
                armControl(armControlUp - armControlDown);
                break;
                
            case STATE_UP:
                if(armControlUp != 0 || armControlDown != 0) { newState(ArmState.STATE_CONTROL); break; }
                if(armDown) { newState(ArmState.STATE_DOWN); break; }
                if(armFold) { newState(ArmState.STATE_FOLD); break; }
                armUp();
                break;
    
            case STATE_DOWN:
                if(armControlUp != 0 || armControlDown != 0) { newState(ArmState.STATE_CONTROL); break; }
                if(armUp) { newState(ArmState.STATE_UP); break; }
                if(armFold) { newState(ArmState.STATE_FOLD); break; }
                armDown();
                break;
    
            case STATE_FOLD:
                if(armControlUp != 0 || armControlDown != 0) { newState(ArmState.STATE_CONTROL); break; }
                if(armUp || armFold) { newState(ArmState.STATE_UP); break; }
                if(armDown) { newState(ArmState.STATE_DOWN); break; }
                armFold();
                break;
        }
    }
    
    private void newState(GripperState newState) { currentGripperState = newState; }
    
    private enum GripperState {
        STATE_OFF,
        STATE_GRIP,
        STATE_LOCK,
        STATE_EJECT
    }
    
    private void newState(ArmState newState) { currentArmState = newState; }
    
    private enum ArmState {
        STATE_CONTROL,
        STATE_UP,
        STATE_DOWN,
        STATE_FOLD
    }
}
