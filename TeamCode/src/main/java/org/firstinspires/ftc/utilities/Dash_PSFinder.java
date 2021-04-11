package org.firstinspires.ftc.utilities;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Dash_PSFinder {

    // RING FINDER
    public static int MAX_Y = 255;
    public static int MIN_Y = 0;
    
    public static int MAX_Cr = 150;
    public static int MIN_Cr = 80;
    
    public static int MAX_Cb = 255;
    public static int MIN_Cb = 70;

    public static int blur = 13;
    public static int erode_const = 5;
    public static int dilate_const = 5;
    public static double horizonLineRatio = 0.0;

}
