package org.firstinspires.ftc.teamcode.Autonomous.lupineAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.Utils;
import org.firstinspires.ftc.teamcode.Vision.GoalFinderPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionUtils;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="GoalFinder", group="Autonomous Linear Opmode")
public class GoalFinder extends LinearOpMode
{
    private GoalFinderPipeline goalFinder = new GoalFinderPipeline();
    public static IMU imu;
    private ButtonControls BC;

    public void initialize(){
        Utils.setOpMode(this);
        imu = new IMU("imu");
        BC = new ButtonControls(gamepad1);
        initVision();
    }

    public void initVision(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionUtils.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        VisionUtils.webcam.setPipeline(new GoalFinderPipeline());
        VisionUtils.webcam.openCameraDeviceAsync(() -> VisionUtils.webcam.startStreaming((int) VisionUtils.IMG_WIDTH, (int) VisionUtils.IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT));
    }

    @Override
    public void runOpMode()
    {

        initialize();

        Utils.multTelemetry.addLine("Waiting for start");
        Utils.multTelemetry.update();
        waitForStart();

        while (opModeIsActive()){
            Utils.multTelemetry.addData("FPS", String.format("%.2f", VisionUtils.webcam.getFps()));
            Utils.multTelemetry.update();
        }
    }
}
