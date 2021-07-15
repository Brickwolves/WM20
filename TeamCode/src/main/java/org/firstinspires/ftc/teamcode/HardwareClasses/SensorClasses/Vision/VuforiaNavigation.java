package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Utilities.OpModeUtils;

public class VuforiaNavigation extends LinearOpMode {

    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackable target;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackableDefaultListener listener;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;

    private String VUFORIA_KEY = "AbuxcJX/////AAABmXadAYnA80uwmb4Rhy4YmvIh7qg/f2yrRu1Nd8O7sSufbUWHSv1jDhunwDBItvFchrvkc8EjTzjh97m2kAPy8YOjBclQbEBtuR8qcIfrGofASCZh2M6vQ0/Au+YbhYh0MLLdNrond+3YjkLswv6+Se3eVGw9y9fPGamiABzIrosjUdanAOWemf8BtuQUW7EqXa4mNPtQ+2jpZQO2sqtqxGu1anHQCD0S/PvdZdB7dRkyWaH6XTZCat5gZ0fpFH/aLWMFP4yiknlgYbjT7gklUAqyDX81pNrQhWWY4dOFnz2WiWhkCt+MNZMLKH5SdsyC7gwKI/r3h51pTwgXZfyYymB60eYAFqEUpeTrL+4LmltN";

    public void initialize(){
        OpModeUtils.setOpMode(this);

        setUpVuforia();
        lastKnownLocation = createMatrix(0,0, 0, 0, 0, 0);
    }

    public void setUpVuforia(){
        parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(parameters);

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("UltimateGoal");
        target = visionTargets.get(0);
        target.setName("Wheels Target");
        target.setLocation(createMatrix(0, 0, 0, 0, 0, 0));
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x, y, z).multiplied(
                Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    public String formatMatrix(OpenGLMatrix matrix){
        return matrix.formatAsTransform();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        visionTargets.activate();

        while (opModeIsActive()){

            OpenGLMatrix latestLocation = listener.getRobotLocation();
            if (latestLocation != null) lastKnownLocation = latestLocation;

            OpModeUtils.multTelemetry.addData("Tracking", target.getName(), listener.isVisible());
            OpModeUtils.multTelemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));
        }

    }
}
