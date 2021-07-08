package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.utilities.IMU;
import org.firstinspires.ftc.utilities.MathUtils;
import org.firstinspires.ftc.utilities.RingBuffer;
import static org.firstinspires.ftc.utilities.Utils.hardwareMap;

public class Gyro {

    private IMU imu;
    private double datum;
    private final RingBuffer<Double> angleRing = new RingBuffer<>(4,0.0);
    private final RingBuffer<Long> angleTimeRing = new RingBuffer<>(4, (long)0);
    
    
    
    private double imuAngle = 0;
    private double rawAngle = 0;
    private double modAngle = 0;
    private double rateOfChange = 0;
    private double rateOfChangeShort = 0;
    
    public Gyro(){
    
    }
    

    public void init() {
        imu = new IMU("imu");
    }
    
    public void update(){
        imuAngle = imu.getAngle();
        rawAngle = imu.getAngle() - datum;
        modAngle = MathUtils.mod(rawAngle, 360);
    
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
    
    public boolean angleRange(double minAngle, double maxAngle){
        minAngle = MathUtils.mod(minAngle, 360);
        maxAngle = MathUtils.mod(maxAngle, 360);
        
        if(maxAngle < minAngle) return modAngle > minAngle || modAngle < maxAngle;
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
}
