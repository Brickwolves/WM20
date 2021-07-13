package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
	
	private final Gamepad gamepad;
	
	public Thumbstick rightStick, leftStick;
	public Button cross, circle, triangle, square, up, down, left, right, RB, LB, RS, LS, share, touchpad;
	public Trigger RT, LT;
	
	
	public Controller(Gamepad gamepad) {
		this.gamepad = gamepad;
		rightStick = new Thumbstick(); leftStick = new Thumbstick();
		
		cross = new Button(); circle = new Button(); triangle = new Button(); square = new Button();
		up = new Button(); down = new Button(); left = new Button(); right = new Button();
		RB = new Button(); LB = new Button(); RS = new Button(); LS = new Button();
		share = new Button(); touchpad = new Button();
		
		RT = new Trigger(); LT = new Trigger();
	}
	
	
	public void update(){
		rightStick.update(gamepad.right_stick_x, gamepad.right_stick_y); leftStick.update(gamepad.left_stick_x, gamepad.left_stick_y);
		
		cross.update(gamepad.cross); circle.update(gamepad.circle); triangle.update(gamepad.triangle); square.update(gamepad.square);
		up.update(gamepad.dpad_up); down.update(gamepad.dpad_down); left.update(gamepad.dpad_left); right.update(gamepad.dpad_right);
		RB.update(gamepad.right_bumper); LB.update(gamepad.left_bumper); RS.update(gamepad.right_stick_button); LS.update(gamepad.left_stick_button);
		share.update(gamepad.share); touchpad.update(gamepad.touchpad);
		
		RT.update(gamepad.right_trigger); LT.update(gamepad.left_trigger);
	}
	
	
	public class Button {
		private boolean hold = false; private boolean press = false; private boolean toggle = false;
		
		private void update(boolean button) {
			boolean wasHeld = hold;
			press = (hold = button) && !wasHeld;
		}
		
		
		public boolean hold() { return hold; }
		
		public boolean press() { return press; }
		
		public boolean toggle() {
			if (press()) toggle = !toggle;
			return (toggle);
		}
	}
	
	
	public class Trigger {
		private final Button trigger = new Button();
		private float value;
		
		private void update(float value) {
			this.value = value;
			trigger.update(hold());
		}
		
		public float value(){ return value; }
		
		public boolean hold() { return value > .7; }
		
		public boolean press() { return trigger.press(); }
		
		public boolean toggle() { return trigger.toggle(); }
		
		public double range(double pressed, double released){
			double range = pressed - released;
			return (value() * range) + released;
		}
		
	}
	
	
	public class Thumbstick {
		private double rawX, rawY, shiftedX, shiftedY;
		
		private void update(Float x, Float y) { rawX = x; rawY = y; }
		
		public double X() { return rawX * -1; }
		
		public double Y() { return rawY * -1; }
		
		
		public void setShift(double shiftAngle) {
			this.shiftedX = (this.rawX * Math.cos(Math.toRadians(shiftAngle))) - (this.rawY * Math.sin(Math.toRadians(shiftAngle)));
			this.shiftedY = (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
		}
		
		public double shiftedX() { return shiftedX * -1; }
		
		public double shiftedY() { return shiftedY * -1; }
		
		
		public double getAngle(){ return ((270 - (Math.atan2(0 - Y(), 0 - X())) * 180 / Math.PI) % 360); }
	}
}
