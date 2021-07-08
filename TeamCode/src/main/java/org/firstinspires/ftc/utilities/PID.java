package org.firstinspires.ftc.utilities;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID {
    private Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    private double proportional;
    private double integral;
    private double derivative;
    private double fComponent;
    private double result = 0;
    public double integralSum = 0;
    private int integralLength;

    private RingBuffer <Double> integralBuffer;
    private RingBuffer<Double> derivitaveBuffer;
    private RingBuffer<Double> timeBuffer;

    private boolean debugMode;
    
    
    public PID(double proportional, double integral, double derivative) { this(proportional, integral, derivative, 0, 0, false); }
    
    public PID(double proportional, double integral, double derivative, double fComponent) { this(proportional, integral, derivative, fComponent, 0, false); }

    public PID(double proportional, double integral, double derivative, int integralLength) { this(proportional, integral, derivative, 0, integralLength, false); }
    
    public PID(double proportional, double integral, double derivative, boolean debugMode) { this(proportional, integral, derivative, 0, 0, debugMode); }
    
    
    public PID(double proportional, double integral, double derivative, double fComponent, int integralLength) { this(proportional, integral, derivative, fComponent, integralLength,false); }
    
    public PID(double proportional, double integral, double derivative, int integralLength, boolean debugMode) { this(proportional, integral, derivative, 0, integralLength, debugMode); }
    
    public PID(double proportional, double integral, double derivative, double fComponent, boolean debugMode) { this(proportional, integral, derivative, fComponent, 0, debugMode); }

    public PID(double proportional, double integral, double derivative, double fComponent, int integralLength, boolean debugMode) {
        this.proportional = proportional;
        this.integral = integral;
        this.derivative = derivative;
        this.fComponent = fComponent;
        this.debugMode = debugMode;
        derivitaveBuffer = new RingBuffer<Double>(3, 0.0);
        timeBuffer = new RingBuffer<Double>(3, 0.0);
        integralBuffer = new RingBuffer<Double>(integralLength, 0.0);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public double update(double error){
    
        integralSum += error;
        if(integralLength != 0){ integralSum -= integralBuffer.getValue(error); }
        
        
        double currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - timeBuffer.getValue(currentTime)) / 1000;
        double deltaError = error - derivitaveBuffer.getValue(error);
        double rateOfChange = (deltaError / deltaTime) / 3;
        
        
        double pComponent = error * proportional;
        double iComponent = integralSum * integral;
        double dComponent = (rateOfChange * derivative);
        
        
        if(debugMode){
            dashboardTelemetry.addData("Proportional", pComponent);
            dashboardTelemetry.addData("Integral", iComponent);
            dashboardTelemetry.addData("Derivative", dComponent);
        }
        
        this.result = pComponent + iComponent + dComponent + fComponent;
        return result;
    }
    
    public double getResult() {
        return result;
    }
    
    public void setFComponent(double fComponent){
        this.fComponent = fComponent;
    }
    
    public void resetIntegralSum(){
        integralSum = 0;
    }
    
    public void setIntegralSum(double integralSum){
        this.integralSum = integralSum;
    }
}
