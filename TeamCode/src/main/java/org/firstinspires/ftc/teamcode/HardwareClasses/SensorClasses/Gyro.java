package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors.Alliance;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.MathUtils;
import org.firstinspires.ftc.utilities.RingBuffer;

public class Gyro {

    private IMU imu;
    private double datum;
    private final RingBuffer<Double> angleRing = new RingBuffer<>(4,0.0);
    private final RingBuffer<Long> angleTimeRing = new RingBuffer<>(4, (long)0);
    
    
    
    private double imuAngle = 0;
    private double rawAngle = 0;
    private double modAngle = 0;
    private double absImuAngle = 0;
    private double absRawAngle = 0;
    private double absModAngle = 0;
    private double rateOfChange = 0;
    private double rateOfChangeShort = 0;
    
    Alliance alliance;
    

    public void init(Alliance alliance) {
        imu = new IMU("imu");
        this.alliance = alliance;
    }
    
    public void init() {
        imu = new IMU("imu");
        this.alliance = null;
    }
    
    public void update(){
        imuAngle = imu.getAngle();
        rawAngle = imu.getAngle() - datum;
        modAngle = MathUtils.mod(rawAngle, 360);
    
        if(alliance == Alliance.RED) {
            absImuAngle = imuAngle + 180;
            absRawAngle = rawAngle + 180;
            absModAngle = MathUtils.mod(absRawAngle, 360);
        }else if(alliance == Alliance.BLUE){
            absImuAngle = imuAngle;
            absRawAngle = rawAngle;
            absModAngle = modAngle;
        }
    
        long currentTime = System.currentTimeMillis();
        long deltaMili = currentTime - angleTimeRing.getValue(currentTime);
        double deltaSeconds = deltaMili / 1000.0;
    
        double currentAngle = Sensors.gyro.rawAngle();
        double deltaAngle = currentAngle - angleRing.getValue(currentAngle);
        
        
        rateOfChange = deltaAngle/deltaSeconds;
        
    }

    public void setImu(IMU imu) {
        this.imu = imu;
    }

    public void setDatum(double datum) {
        this.datum = datum;
    }
    
    public void reset() { datum = imu.getAngle(); }

    public double rawAngle() {
        return rawAngle;
    }
    
    public double IMUAngle() {
        return imuAngle;
    }

    public double modAngle() {
        return modAngle;
    }
    
    public boolean angleRange(double minAngle, double maxAngle) {
        minAngle = MathUtils.mod(minAngle, 360);
        maxAngle = MathUtils.mod(maxAngle, 360);
        
        if (maxAngle < minAngle) return modAngle > minAngle || modAngle < maxAngle;
        else return modAngle > minAngle && modAngle < maxAngle;
    }
    
    
    public double rateOfChange(){
        return rateOfChange;
    }
    
    public double rateOfChangeShort(){
        return rateOfChangeShort;
    }

    //TODO Make this work with more accuracy. Curse you, floorMod(int)!
    @RequiresApi(api = Build.VERSION_CODES.N)
    public double absToRel(int targetAbsoluteAngle){
        double retval = MathUtils.floorModDouble((360) + imu.getAngle(), 360);
        return (retval <= 180) ? retval : -1 * (360 - retval);
    }
    
    public double absRawAngle() {
        return absRawAngle;
    }
    
    public double absIMUAngle() {
        return absImuAngle;
    }
    
    public double absModAngle() {
        return absModAngle;
    }
    
    
    public boolean absAngleRange(double minAngle, double maxAngle){
        minAngle = MathUtils.mod(minAngle, 360);
        maxAngle = MathUtils.mod(maxAngle, 360);
        
        if(maxAngle < minAngle) return absModAngle > minAngle || absModAngle < maxAngle;
        else return absModAngle > minAngle && absModAngle < maxAngle;
    }
    
}
