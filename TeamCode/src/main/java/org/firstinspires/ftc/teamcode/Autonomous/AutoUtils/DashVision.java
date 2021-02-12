package org.firstinspires.ftc.teamcode.Autonomous.AutoUtils;
import com.acmerobotics.dashboard.config.Config;

@Config
public class DashVision {
    /* ------- VISION --------- */
    // Top rectangle starting percentages
    public static double rectTopX1Percent = 0.62; public static double rectTopX2Percent = 0.84;
    public static double rectTopY1Percent = 0.5; public static double rectTopY2Percent = .58;

    // Bottom rectangle starting percentages
    public static double rectBottomX1Percent = 0.62; public static double rectBottomX2Percent = 0.84;
    public static double rectBottomY1Percent = 0.58; public static double rectBottomY2Percent = 0.62;

    // Generally, orange should be around 90-100
    public static double orangeMax = 110;
    public static double orangeMin = 80;

    public static double diagnostic_ring_count = 0.0;
}