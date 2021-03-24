package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller {
	
	
	private final Gamepad gamepad;
	private boolean isSq;
	private boolean isTr;
	private boolean isCi;
	private boolean isCr;
	private boolean isUp;
	private boolean isLeft;
	private boolean isDown;
	private boolean isRight;
	private boolean isLS;
	private boolean isRS;
	private boolean isLB;
	private boolean isRB;
	private boolean isLT;
	private boolean isRT;
	private boolean toggleSq = false;
	private boolean toggleTr = false;
	private boolean toggleCr = false;
	private boolean toggleCi = false;
	private boolean toggleUp = false;
	private boolean toggleDown = false;
	private boolean toggleLeft = false;
	private boolean toggleRight = false;
	private boolean toggleLS = false;
	private boolean toggleRS = false;
	private boolean toggleLB = false;
	private boolean toggleRB = false;
	private boolean toggleLT = false;
	private boolean toggleRT = false;
	private boolean pressSq = false;
	private boolean pressTr = false;
	private boolean pressCr = false;
	private boolean pressCi = false;
	private boolean pressUp = false;
	private boolean pressDown = false;
	private boolean pressLeft = false;
	private boolean pressRight = false;
	private boolean pressLS = false;
	private boolean pressRS = false;
	private boolean pressLB = false;
	private boolean pressRB = false;
	private boolean pressLT = false;
	private boolean pressRT = false;
	
	
	public Controller(Gamepad gamepad) {
		this.gamepad = gamepad;
	}
	
	public Thumbstick getRightThumbstick() {
		return new Controller.Thumbstick(gamepad.right_stick_x, gamepad.right_stick_y);
	}
	
	public Thumbstick getLeftThumbstick() {
		return new Controller.Thumbstick(gamepad.left_stick_x, gamepad.left_stick_y);
	}
	
	public void update(){
		boolean wasCr = isCr;
		pressCr = (isCr = gamepad.cross) && !wasCr;
		
		boolean wasCi = isCi;
		pressCi = (isCi = gamepad.circle) && !wasCi;
		
		boolean wasSq = isSq;
		pressSq = (isSq = gamepad.square) && !wasSq;
		
		boolean wasTr = isTr;
		pressTr = (isTr = gamepad.triangle) && !wasTr;
		
		boolean wasUp = isUp;
		pressUp = (isUp = gamepad.dpad_up) && !wasUp;
		
		boolean wasDown = isDown;
		pressDown = (isDown = gamepad.dpad_down) && !wasDown;
		
		boolean wasLeft = isLeft;
		pressLeft = (isLeft = gamepad.dpad_left) && !wasLeft;
		
		boolean wasRight = isRight;
		pressRight = (isRight = gamepad.dpad_right) && !wasRight;
		
		boolean wasLS = isLS;
		pressLS = (isLS = gamepad.left_stick_button) && !wasLS;
		
		boolean wasRS = isRS;
		pressRS = (isRS = gamepad.right_stick_button) && !wasRS;
		
		boolean wasLB = isLB;
		pressLB = (isLB = gamepad.left_bumper) && !wasLB;
		
		boolean wasRB = isRB;
		pressRB = (isRB = gamepad.right_bumper) && !wasRB;
		
		boolean wasLT = isLT;
		pressLT = (isLT = LT()) && !wasLT;
		
		boolean wasRT = isRT;
		pressRT = (isRT = RT()) && !wasRT;
	}
	
	//cross
	public boolean cross() {
		return gamepad.a;
	}
	
	public boolean crossPress() {
		return pressCr;
	}
	
	public boolean crossToggle() {
		if (crossPress()) {
			toggleCr = !toggleCr;
		}
		return (toggleCr);
	}
	
	
	//circle
	public boolean circle() {
		return gamepad.b;
	}
	
	public boolean circlePress() {
		return pressCi;
	}
	
	public boolean circleToggle() {
		if (circlePress()) {
			toggleCi = !toggleCi;
		}
		return (toggleCi);
	}
	
	
	//square
	public boolean square() {
		return gamepad.x;
	}
	
	public boolean squarePress() {
		return pressSq;
	}
	
	public boolean squareToggle() {
		if (squarePress()) {
			toggleSq = !toggleSq;
		}
		return (toggleSq);
	}
	
	
	//triangle
	public boolean triangle() {
		return gamepad.y;
	}
	
	public boolean trianglePress() {
		return pressTr;
	}
	
	public boolean triangleToggle() {
		if (trianglePress()) {
			toggleTr = !toggleTr;
		}
		return (toggleTr);
	}
	
	
	//dpad up
	public boolean up() {
		return gamepad.dpad_up;
	}
	
	public boolean upPress() {
		return pressUp;
	}
	
	public boolean upToggle() {
		if (upPress()) {
			toggleUp = !toggleUp;
		}
		return (toggleUp);
	}
	
	
	//dpad down
	public boolean down() {
		return gamepad.dpad_down;
	}
	
	public boolean downPress() {
		return pressDown;
	}
	
	public boolean downToggle() {
		boolean wasDown = isDown;
		if (downPress()) {
			toggleDown = !toggleDown;
		}
		return (toggleDown);
	}
	
	
	//dpad left
	public boolean left() {
		return gamepad.dpad_left;
	}
	
	public boolean leftPress() {
		return pressLeft;
	}
	
	public boolean leftToggle() {
		boolean wasLeft = isLeft;
		if (leftPress()) {
			toggleLeft = !toggleLeft;
		}
		return (toggleLeft);
	}
	
	
	//dpad right
	public boolean right() {
		return gamepad.dpad_right;
	}
	
	public boolean rightPress() {
		return pressRight;
	}
	
	public boolean rightToggle() {
		if (rightPress()) {
			toggleRight = !toggleRight;
		}
		return (toggleRight);
	}
	
	
	//left stick button
	public boolean LS() {
		return gamepad.left_stick_button;
	}
	
	public boolean LSPress() {
		return pressLS;
	}
	
	public boolean LSToggle() {
		boolean wasLS = isLS;
		if (LSPress()) {
			toggleLS = !toggleLS;
		}
		return (toggleLS);
	}
	
	
	//right stick button
	public boolean RS() {
		return gamepad.right_stick_button;
	}
	
	public boolean RSPress() {
		return pressRS;
	}
	
	public boolean RSToggle() {
		if (RSPress()) {
			toggleRS = !toggleRS;
		}
		return (toggleRS);
	}
	
	
	//left bumper
	public boolean LB() {
		return gamepad.left_bumper;
	}
	
	public boolean LBPress() {
		return pressLB;
	}
	
	public boolean LBToggle() {
		if (LBPress()) {
			toggleLB = !toggleLB;
		}
		return (toggleLB);
	}
	
	
	//right bumper
	public boolean RB() {
		return gamepad.right_bumper;
	}
	
	public boolean RBPress() {
		return pressRB;
	}
	
	public boolean RBToggle() {
		if (RBPress()) {
			toggleRB = !toggleRB;
		}
		return (toggleRB);
	}
	
	
	//left trigger
	public float LTFloat() {
		return gamepad.left_trigger;
	}
	
	public boolean LT() { return gamepad.left_trigger > .4; }
	
	public boolean LTPress(){
		return pressLT;
	}
	
	public boolean LTToggle() {
		if (LTPress()) {
			toggleLT = !toggleLT;
		}
		return toggleLT;
	}
	
	
	
	
	//right trigger
	public float RTFloat() {
		return gamepad.right_trigger;
	}
	
	public boolean RT() { return gamepad.right_trigger > .4; }
	
	public boolean RTPress(){
		return pressRT;
	}
	
	public boolean RTToggle() {
		if (RTPress()) {
			toggleRT = !toggleRT;
		}
		return (toggleRT);
	}
	
	
	
	public boolean options() {
		return gamepad.options;
	}
	
	
	public boolean share() {
		return gamepad.share;
	}
	
	
	public boolean touchpad() { return gamepad.touchpad; }
	
	
	public class Thumbstick {
		
		private final double rawX;
		private final double rawY;
		private double shiftedX;
		private double shiftedY;
		
		public Thumbstick(Double x, Double y) {
			this.rawX = x;
			this.rawY = y;
		}
		
		public Thumbstick(Float x, Float y) {
			this.rawX = x;
			this.rawY = y;
		}
		
		public boolean isInput() {
			return (getX() != 0) || (getY() != 0);
		}
		
		public double getX() {
			return rawX;
		}
		
		public double getY() {
			return rawY;
		}
		
		public void setShift(double shiftAngle) {
			this.shiftedX = (this.rawX * Math.cos(Math.toRadians(shiftAngle))) - (this.rawY * Math.sin(Math.toRadians(shiftAngle)));
			this.shiftedY = (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
		}
		
		public double getShiftedX() {
			return shiftedX;
		}
		
		public double getShiftedY() {
			return shiftedY;
		}
		
		public double getShiftedX(Double shiftAngle) {
			return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
		}
		
		public double getShiftedY(Double shiftAngle) {
			return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
		}
		
		public double getInvertedX() {
			return rawX * -1;
		}
		
		public double getInvertedY() {
			return rawY * -1;
		}
		
		public double shiftedX() {
			return shiftedX * -1;
		}
		
		public double shiftedY() {
			return shiftedY * -1;
		}
		
		public double shiftedX(Double shiftAngle) {
			return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle))) * -1;
		}
		
		public double shiftedY(Double shiftAngle) {
			return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle))) * -1;
		}
		
		public double getAngle(){
			return ((270 - (Math.atan2(0 - getInvertedY(), 0 - getInvertedX())) * 180 / Math.PI) % 360);
		}
		
		
	}
}
