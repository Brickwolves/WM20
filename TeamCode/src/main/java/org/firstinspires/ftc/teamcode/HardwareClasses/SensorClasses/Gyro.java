package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.MathUtils;
import org.firstinspires.ftc.utilities.RingBuffer;

public class Gyro {

    private IMU imu;
    private double datum;
    private final RingBuffer<Double> timeRing = new RingBuffer<>(4, 0.0);
    private final RingBuffer<Double> angleRing = new RingBuffer<>(4, 0.0);
    
    private double imuAngle = 0;
    private double rawAngle = 0;
    private double modAngle = 0;

    public Gyro(IMU imu, double datum) {
        this.imu = imu;
        this.datum = datum;
    }
    
    public void update(){
        imuAngle = imu.getAngle();
        rawAngle = imu.getAngle() - datum;
        modAngle = MathUtils.mod(rawAngle, 360);
    }

    public void setImu(IMU imu) {
        this.imu = imu;
    }

    public void setDatum(double datum) {
        this.datum = datum;
    }
    
    public void reset() { datum = imu.getAngle(); }

    public double getRawAngle() {
        return rawAngle;
    }
    
    public double getIMUAngle() {
        return imuAngle;
    }

    public double getModAngle() {
        return modAngle;
    }
    
    public boolean angleRange(double minAngle, double maxAngle){
        minAngle = MathUtils.mod(minAngle, 360);
        maxAngle = MathUtils.mod(maxAngle, 360);
        
        if(maxAngle < minAngle) return modAngle > minAngle || modAngle < maxAngle;
        else return modAngle > minAngle && modAngle < maxAngle;
        
        
    }
    
    public double rateOfChange(){
        double retVal;

        double currentTime = System.currentTimeMillis();
        double deltaMili = currentTime - timeRing.getValue(currentTime);
        double deltaSeconds = deltaMili / 1000.0;

        double currentAngle = getRawAngle();
        double deltaAngle = currentAngle - angleRing.getValue(currentAngle);

        retVal = deltaAngle / deltaSeconds;

        return retVal;
    }

    //TODO Make this work with more accuracy. Curse you, floorMod(int)!
    public double absToRel(int targetAbsoluteAngle){
        double retval = MathUtils.floorModDouble((360) + imu.getAngle(), 360);
        return (retval <= 180) ? retval : -1 * (360 - retval);
    }
}
