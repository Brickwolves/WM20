package org.firstinspires.ftc.teamcode.Autonomous.lupineAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.Autonomous.lupineAutos.Utils.multTelemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareClasses.SensorClasses.RingFinderPipeline;
import org.firstinspires.ftc.teamcode.HardwareClasses.Sensors;
import org.firstinspires.ftc.utilities.IMU;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



@Autonomous(name="RingFinder", group="Autonomous Linear Opmode")
public class RingFinder extends LinearOpMode
{
    private RingFinderPipeline ringFinder = new RingFinderPipeline();
    private static Sensors sensors;

    public void initialize(){
        Utils.setOpMode(this);
        sensors = new Sensors(new IMU("imu"), null, null,null, null);
        initVision();
    }

    public void initVision(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionUtils.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        VisionUtils.webcam.setPipeline(ringFinder);
        VisionUtils.webcam.openCameraDeviceAsync(() -> VisionUtils.webcam.startStreaming((int) VisionUtils.IMG_WIDTH, (int) VisionUtils.IMG_HEIGHT, OpenCvCameraRotation.UPRIGHT));
    }

    @Override
    public void runOpMode()
    {

        initialize();

        multTelemetry.addLine("Waiting for start");
        multTelemetry.update();
        waitForStart();

        while (opModeIsActive()){
            multTelemetry.addData("Ring Count", ringFinder.getRingCount());
            multTelemetry.addData("FPS", String.format("%.2f", VisionUtils.webcam.getFps()));
            multTelemetry.update();
        }
    }
}
