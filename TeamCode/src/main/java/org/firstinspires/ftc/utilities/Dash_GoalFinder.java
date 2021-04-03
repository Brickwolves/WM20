package org.firstinspires.ftc.utilities;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Dash_GoalFinder {

    // GOAL FINDER
    public static int MIN_H = 100;
    public static int MAX_H = 120;

    public static int MIN_S = 165;
    public static int MAX_S = 255;

    public static int MIN_V = 90;
    public static int MAX_V = 200;

    public static int blur = 5;
    public static int erode_const = 5;
    public static int dilate_const = 5;
    public static int goalWidth = 100;
    public static double horizonLineRatio = 0.5;

}
