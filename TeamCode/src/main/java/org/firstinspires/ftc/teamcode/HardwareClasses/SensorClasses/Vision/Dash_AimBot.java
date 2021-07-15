package org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.Vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Dash_AimBot {

    // GOAL FINDER
    public static int MIN_H = 100;
    public static int MAX_H = 120;

    public static int MIN_S = 110;
    public static int MAX_S = 255;

    public static int MIN_V = 90;
    public static int MAX_V = 220;

    public static int[] MAX_HSV = {0, 0, 0};
    public static int[] MIN_HSV = {0, 0, 0};
    public static int[] MARGINS = {25, 90, 50};

    public static int blur = 5;
    public static int erode_const = 5;
    public static int dilate_const = 5;
    public static int goalWidth = 100;
    public static double horizonLineRatio = 1;

    // Debugging tools
    public static boolean DEBUG_MODE_ON     = true;
    public static boolean INIT_COMPLETED = true;
    public static int INIT_RECT_SIDELENGTH = 10;
    public static

}
