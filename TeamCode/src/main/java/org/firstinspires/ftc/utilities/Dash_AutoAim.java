package org.firstinspires.ftc.utilities;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Dash_AutoAim {

    // GOAL FINDER
    public static int MIN_H = 97;
    public static int MAX_H = 122;

    public static int MIN_S = 155;
    public static int MAX_S = 255;

    public static int MIN_V = 87;
    public static int MAX_V = 203;
    

    

    public static int blur = 5;
    public static int erode_const = 5;
    public static int dilate_const = 5;
    public static int goalWidth = 100;
    public static double horizonLineRatio = .7;

}
