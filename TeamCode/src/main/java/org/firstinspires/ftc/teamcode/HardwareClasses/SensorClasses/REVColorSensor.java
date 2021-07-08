package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class REVColorSensor {
	private ColorSensor sensorColor;
	private final float[] hsvValues = {0F, 0F, 0F};
	private final float[] values = hsvValues;
	private final double SCALE_FACTOR = 255;
	
	private double red, blue, green;
	
	public REVColorSensor(ColorSensor sensorColor){
		this.sensorColor = sensorColor;
	}
	
	public void update(){
		red = sensorColor.red();
	}
	
	public float hue(){ return hsvValues[0]; }
	
	public float saturation(){ return hsvValues[1]; }
	
	public float value(){ return hsvValues[2]; }
	
	public double red(){ return red; }
	
	public double blue(){ return blue; }
	
	public double green(){ return green; }
}
