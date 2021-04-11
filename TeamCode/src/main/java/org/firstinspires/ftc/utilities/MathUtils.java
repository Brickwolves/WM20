package org.firstinspires.ftc.utilities;

import android.os.Build;

import androidx.annotation.RequiresApi;

import static java.lang.Math.abs;
import static java.lang.Math.floorMod;

public class MathUtils {

    public static double sigmoid(double rawNumber, double range, double outputRange){
        return (outputRange / (1 + Math.pow(range, -rawNumber))) - outputRange / 2;
    }

    public static double degsToRads(Double degs){
        return degs * (Math.PI / 180);
    }


    public static double mod(double value, int base){
        return (value % base + base) % base;
    }
    
    public static double pow(double value, double exponent) {
        if(value == 0) return 0;
        else return Math.pow(value, exponent) * (value / abs(value));
    }

    //TODO  I need to do a lot more documentation on my code as I write it -  so I can do a better job of maintenance
    //TODO Make these actually have input for accuracy like a normal function
    @RequiresApi(api = Build.VERSION_CODES.N)
    public static double floorModDouble(double dividend, double divisor){
        return floorMod(Math.round(dividend * 1e6), Math.round(divisor * 1e6)) / 1e6;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static double floorModDouble(double dividend, int divisor){
        return floorMod(Math.round(dividend * 1e6), divisor) / 1e6;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public static double floorModDouble(int dividend, double divisor){
        return floorMod(dividend, Math.round(divisor * 1e6)) / 1e6;

    }


}
