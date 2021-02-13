package org.firstinspires.ftc.utilities;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID {
    private Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

    private double proportional;
    private double integral;
    private double derivative;
    private double result = 0;
    private double integralSum = 0;
    private double previousError = 0;
    private long previousTime;
    private int integralLength;

    private PIDRingBuffer errors;
    private RingBuffer<Double> integralSumBuffer;
    private RingBuffer<Double> timeBuffer;

    private boolean debugMode;

    public PID(double proportional, double integral, double derivative, int integralLength) {
        this(proportional, integral, derivative, integralLength, false);
    }

    public PID(double proportional, double integral, double derivative, int integralLength, boolean debugMode) {
        this.proportional = proportional;
        this.integral = integral;
        this.derivative = derivative;
        this.debugMode = debugMode;
        this.integralLength = integralLength;
        integralSumBuffer = new RingBuffer<Double>(integralLength, 0.0);
        timeBuffer = new RingBuffer<Double>(integralLength, 0.0);
    }

    public Double update(double error){
        double iComponent;
        if(integralLength == 0) {
            integralSum += error;
            iComponent = integralSum * integral;
        }else{
            integralSum = integralSum + error - integralSumBuffer.getValue(error);
            iComponent = integralSum * integral / integralLength;
        }
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - previousTime) / 1000.0;
        double rateOfChange = (error - previousError) / deltaTime;
        previousTime = currentTime;
        previousError = error;
        double pComponent = error * proportional;
        double dComponent = (rateOfChange * derivative);
        if(debugMode){
            dashboardTelemetry.addData("Proportional", pComponent);
            dashboardTelemetry.addData("Integral", iComponent);
            dashboardTelemetry.addData("Derivative", dComponent);
        }
        this.result = pComponent + iComponent + dComponent;
        return result;
    }
    public double getResult() {
        return result;
    }
    public void resetIntegralSum(){
        integralSum = 0;
    }
}
