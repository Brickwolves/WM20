package org.firstinspires.ftc.teamcode.HardwareClasses;import android.os.Build;import androidx.annotation.RequiresApi;import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.hardware.DcMotorSimple;import org.firstinspires.ftc.utilities.GyroUtils;import org.firstinspires.ftc.utilities.PID;import org.firstinspires.ftc.utilities.RingBuffer;import org.firstinspires.ftc.utilities.RingBufferOwen;import static java.lang.Math.abs;import static java.lang.Math.floorMod;public class MecanumChassis {    public DcMotor frontLeft, frontRight, backLeft, backRight;    private PID telePID = new PID(.015, 0.0000, .0004, 20);    private PID autoPID = new PID(.02, 0.0000, .0004, 20);    private final RingBufferOwen timeRing = new RingBufferOwen(20);    private final RingBuffer<Double> positionRingFL = new RingBuffer<Double>(20, 0.0);    private final RingBuffer<Double> positionRingFR = new RingBuffer<Double>(20, 0.0);    private final RingBuffer<Double> positionRingBL = new RingBuffer<Double>(20, 0.0);    private final RingBuffer<Double> positionRingBR = new RingBuffer<Double>(20, 0.0);        private DriveState currentDriveState = DriveState.STATE_FULL_CONTROL;        private Gyro gyro;    private double drive, strafe, turn, power, targetAngle, bigTurn;    private double closestTarget = 0;    private double previousTarget = 0;    private double turnTime = 0;    private double turnYInt = 0;    private double unique = 238208;    public double currentTicks = 0;    public boolean isStrafeFinished = false;    private double adjustmentAngle = 0;    private boolean fullControlEnabledPrev = false;    private final static double ACCEL_RATE = 0.001;    private static final double TELE_ACCEL = .002;    //constructor for MecanumChassis. Sets all motors to correct direction and sets them to run with encoders. Also sets target angle and saves all motors.    public MecanumChassis(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gyro gyro, Double targetAngle) {        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);        backRight.setDirection(DcMotorSimple.Direction.FORWARD);        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);                this.frontLeft = frontLeft;        this.frontRight = frontRight;        this.backLeft = backLeft;        this.backRight = backRight;        this.gyro = gyro;        this.targetAngle = targetAngle;    }    //this is a copy of MecanumChassis except it automatically sets targetAngle to 0    public MecanumChassis(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, Gyro gyro) {        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);        backRight.setDirection(DcMotorSimple.Direction.FORWARD);        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        this.frontLeft = frontLeft;        this.frontRight = frontRight;        this.backLeft = backLeft;        this.backRight = backRight;        this.gyro = gyro;        this.targetAngle = 0;    }    public void resetGyro(){        gyro.setDatum(gyro.getRawAngle());        targetAngle = 0;    }    @RequiresApi(api = Build.VERSION_CODES.N)    public double closestTarget(double targetAngle){        double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + gyro.getRawAngle()) * 1e6), Math.round(360.000 * 1e6)) / 1e6;        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);        return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? gyro.getRawAngle() - simpleTargetDelta : gyro.getRawAngle() - alternateTargetDelta;    }    //mostly depreciated    public void setPower(double drive, double strafe, double turn, double power){        this.drive = drive;        this.strafe = strafe;        this.turn = turn;        this.power = power;                double flPower = (this.drive - this.strafe - this.turn) * this.power;        double frPower = (this.drive + this.strafe + this.turn) * this.power;        double blPower = (this.drive + this.strafe - this.turn) * this.power;        double brPower = (this.drive - this.strafe + this.turn) * this.power;        double maxPower = Math.abs(Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower))));            if (maxPower > 1) {            flPower = flPower / maxPower;            frPower = frPower / maxPower;            blPower = blPower / maxPower;            brPower = brPower / maxPower;        }                if(maxPower < .05 && maxPower > -.05) {            flPower = 0;            frPower = 0;            blPower = 0;            brPower = 0;        }            this.frontLeft.setPower(flPower);        this.frontRight.setPower(frPower);        this.backLeft.setPower(blPower);        this.backRight.setPower(brPower);            }    public void setPowerTele(double drive, double strafe, double turn, double power){        double currentTime = System.currentTimeMillis();        double deltaTime = currentTime - turnTime;        double inputTurn;                if (turn != 0) {            inputTurn = turn;            bigTurn = turn;            turnTime = currentTime;            turnYInt = bigTurn;            targetAngle = gyro.getRawAngle();        }else {            if(bigTurn > .1) {                bigTurn = Math.max(0, -TELE_ACCEL * deltaTime + turnYInt);                targetAngle = gyro.getRawAngle();            }else if(bigTurn < -.1) {                bigTurn = Math.min(0, TELE_ACCEL * deltaTime + turnYInt);                targetAngle = gyro.getRawAngle();            }else{                bigTurn = 0;            }           inputTurn = telePID.update((targetAngle - gyro.getRawAngle()) / ((getRPM() / 600) + .5));                    }        setPower(drive, strafe, inputTurn, power);    }    public void setPowerTele(double drive, double strafe, double turn){        setPowerTele(drive, strafe, turn,1.0);    }        public void setPowerAuto(double drive, double strafe, double targetAngle, double power){        this.targetAngle = targetAngle;        turn = autoPID.update(this.targetAngle - gyro.getRawAngle());        setPower(drive, strafe, turn, power);    }        public void strafe(double distance, double heading, double strafeAngle, double targetPower, double startPower, double endPower, int unique){                distance = Math.abs(distance);        if(startPower == 0){            startPower = .05;        }        startPower = Math.abs(startPower);        targetPower = Math.abs(targetPower);        endPower = Math.abs(endPower);            if (this.unique != unique){            resetMotors();            this.unique = unique;        }            double currentAngle = gyro.getRawAngle();        double currentDistance = frontLeft.getCurrentPosition();        double remainingDistance = distance - currentDistance;                double accelRate = 0.003;        double acceleratePower = Math.sqrt(accelRate * (currentDistance + 1/accelRate * Math.pow(startPower, 2)));        double deceleratePower = Math.sqrt(accelRate * (remainingDistance + 1/accelRate * Math.pow(endPower, 2)));        double currentPower = Math.min(Math.min(acceleratePower, deceleratePower), targetPower);            double drive = Math.cos(strafeAngle - currentAngle);        double strafe = Math.sin(strafeAngle - currentAngle);        double turn = autoPID.update(heading - currentAngle);            isStrafeFinished = currentDistance >= distance;        currentTicks = currentDistance;                setPower(drive,strafe,turn,currentPower);            }    public double adjustedTicks() {        double measuredTicks = ((Math.abs(frontRight.getCurrentPosition()) + Math.abs(frontLeft.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition()) + Math.abs(backLeft.getCurrentPosition())) / 4.0);        double angleAdjustment = (Math.abs(Math.abs(drive) - Math.abs(strafe)) / power - 1) * -1;        return Math.sqrt(Math.pow(measuredTicks, 2) + Math.pow(measuredTicks * angleAdjustment, 2));    }        public void gyroSteering(double targetAngle, double power){        setPowerAuto(power,0, autoPID.update(targetAngle - gyro.getRawAngle()), 1);    }        public void resetMotors(){        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);    }        public double getRPM(){        double retVal;                long currentTime = System.currentTimeMillis();        long deltaMili = currentTime - timeRing.getValue(currentTime);        double deltaMinutes = deltaMili / 60000.0;                double currentPositionFL = frontLeft.getCurrentPosition();        double currentPositionFR = frontRight.getCurrentPosition();        double currentPositionBL = backLeft.getCurrentPosition();        double currentPositionBR = backRight.getCurrentPosition();                double deltaRotationsFL = Math.abs(currentPositionFL - positionRingFL.getValue(currentPositionFL)) / 537.6;        double deltaRotationsFR = Math.abs(currentPositionFR - positionRingFR.getValue(currentPositionFR)) / 537.6;        double deltaRotationsBL = Math.abs(currentPositionBL - positionRingBL.getValue(currentPositionBL)) / 537.6;        double deltaRotationsBR = Math.abs(currentPositionBR - positionRingBR.getValue(currentPositionBR)) / 537.6;                retVal = ((deltaRotationsFL + deltaRotationsFR + deltaRotationsBL + deltaRotationsBR) / 4.0) / deltaMinutes;                return retVal;    }    //sets power for all the motors as needed via setPowerTele and setPowerAuto calls; there are two modes in this method set by whether flyByWire is true: Fly By Wire mode, where all rotation is controlled by the PID system based on targetAngle, and Full Control mode, where the thumbstick can rotate the robot directly when being moved and uses targetAngle when it is not being moved    @RequiresApi(api = Build.VERSION_CODES.N)    public void driveState(double drive, double strafe, double turn, double power, boolean flyByWire){        power = (((power + 1) / -2) + 1.5);        //FLY BY WIRE MODE: this is the state where the user inputs set the targetAngle, and then only the PID code is adjusting the rotation code        if (flyByWire) {            setPowerAuto(drive, strafe, targetAngle, power);        //FULL CONTROL MODE: this is the state where the user can use the thumbstick for small rotation adjustments        } else {            //if the turn thumbstick is not in the neutral positon, power is set manually, and a flag indicating that full control was enabled on the last cycle is set to true            if (turn != 0) {                setPowerTele(drive, strafe, turn, power);                fullControlEnabledPrev = true;                //if the thumbstick is neutral AND the last cycle was a fullControl cycle, then the targetAngle is set to the gyro angle, setPowerAuto is executed, and the full control flag is reset to false            } else if (fullControlEnabledPrev) {                targetAngle = gyro.getRawAngle();                setPowerAuto(drive, strafe, targetAngle, power);                fullControlEnabledPrev = false;                //if the last cycle was not full control, and the thumbstick is not neutral, then it just executes a setPowerAuto call with targetAngle            } else {                setPowerAuto(drive, strafe, targetAngle, power);            }        }    }    @RequiresApi(api = Build.VERSION_CODES.N)    //if the boolean is true, it sets the targetAngle for the PID system to the closest coterminal angle to the input's respective angle (North 0, East 270, South 180, West 90) with priority being in that order    public void setCardinalAngle(boolean northInput, boolean eastInput, boolean southInput, boolean westInput){        if (northInput) {            targetAngle = closestTarget(0);        }        if (eastInput) {            targetAngle = closestTarget(270);        }        if (southInput) {            targetAngle = closestTarget(180);        }        if (westInput) {            targetAngle = closestTarget(90);        }    }    //sets the targetAngle for the PID system to the angle the input thumbstick is being pushed in if it is beyond a .1 deadzone    @RequiresApi(api = Build.VERSION_CODES.N)    public void setTargetAngle(Controller.Thumbstick thumbstickAngleInput){        if(abs(thumbstickAngleInput.getX()) > .1 || abs(thumbstickAngleInput.getY()) > .1){            targetAngle = closestTarget(thumbstickAngleInput.getAngle());        }    }    /*    public void adjustmentState(boolean adjRight, boolean adjLeft, double deltaAngle){        if(adjRight) { newState(DriveState.STATE_ADJUSTMENT); setAdjustmentAngle(gyro.getRawAngle() - deltaAngle); }        if(adjLeft) { newState(DriveState.STATE_ADJUSTMENT); setAdjustmentAngle(gyro.getRawAngle() + deltaAngle); }    }     */        public double getAdjustmentAngle(){ return adjustmentAngle; }        public void setAdjustmentAngle(double adjustmentAngle){ this.adjustmentAngle = adjustmentAngle; }        private void newState(DriveState newState) {        currentDriveState = newState;    }        private enum DriveState {        STATE_FULL_CONTROL,        STATE_FLY_BY_WIRE    }    }