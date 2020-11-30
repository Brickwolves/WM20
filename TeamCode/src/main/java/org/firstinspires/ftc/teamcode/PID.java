package org.firstinspires.ftc.teamcode;

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

    private org.firstinspires.ftc.teamcode.PIDRingBuffer errors;

    private boolean debugMode;

    public PID(double proportional, double integral, double derivative, int integralLength) {
        this(proportional, integral, derivative, integralLength, false);
    }

    public PID(double proportional, double integral, double derivative, int integralLength, boolean debugMode) {
        this.proportional = proportional;
        this.integral = integral;
        this.derivative = derivative;
        this.debugMode = debugMode;
        errors = new org.firstinspires.ftc.teamcode.PIDRingBuffer(integralLength);
        previousTime = System.currentTimeMillis();
    }

    public Double update(double error){
        org.firstinspires.ftc.teamcode.PIDRingBuffer.IntegralDerivativePair integralDerivativePair = errors.update(error, System.currentTimeMillis());
        integralSum += error;
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - previousTime) / 1000.0;
        double rateOfChange = (error - previousError) / deltaTime;
        previousTime = currentTime;
        previousError = error;
        double pComponent = error * proportional;
        double iComponent = integralSum * integral;
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
}
