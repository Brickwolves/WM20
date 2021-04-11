package org.firstinspires.ftc.teamcode.HardwareClasses;import android.os.Build;import androidx.annotation.RequiresApi;import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.hardware.DcMotorSimple;import com.qualcomm.robotcore.util.Range;import org.firstinspires.ftc.utilities.PID;import org.firstinspires.ftc.utilities.RingBuffer;import org.firstinspires.ftc.utilities.RingBufferOwen;import static java.lang.Math.abs;import static java.lang.Math.max;import static java.lang.Math.PI;import static java.lang.Math.min;import static java.lang.Math.floorMod;public class MecanumChassis {    public DcMotor frontLeft, frontRight, backLeft, backRight;    private final PID telePID = new PID(.02, 0, .004, 8);    private final PID autoPID = new PID(.02, 0, .003, 8);    private final PID visionPID = new PID(0.03, 0.0003, .007, 10);    private final RingBufferOwen timeRing = new RingBufferOwen(20);    private final RingBuffer<Double> positionRingFL = new RingBuffer<Double>(20, 0.0);    private final RingBuffer<Double> positionRingFR = new RingBuffer<Double>(20, 0.0);    private final RingBuffer<Double> positionRingBL = new RingBuffer<Double>(20, 0.0);    private final RingBuffer<Double> positionRingBR = new RingBuffer<Double>(20, 0.0);        private final RingBuffer<Double> angleRing = new RingBuffer<>(4,0.0);    private final RingBuffer<Long> angleTimeRing = new RingBuffer<>(4, (long)0);        private DriveState currentDriveState = DriveState.STATE_FULL_CONTROL;    private double drive = 0;    private double strafe = 0;    private double turn = 0;    private double power = 1;    private double targetAngle, releaseAngle = 0;    private double adjRateOfChange = 0;        private double adjustmentAngle = 0;    private double towerAimAngle = 0;    private double PSAngle = 0;    private double PS2Angle = 0;    private double PS3Angle = 0;    double startAngle = 0;    public double strafeDrive = 0;    public double autoDrive = 0;    public double autoStrafe = 0;    public double autoTurn = 0;    public double autoPower = 0;    public double hmmm = 0;        public double currentInches = 0;    public boolean isStrafeComplete = true;    public boolean isTurnComplete = true;    private int powerShotState = 3;        public MecanumChassis(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);        backRight.setDirection(DcMotorSimple.Direction.FORWARD);        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        this.frontLeft = frontLeft;        this.frontRight = frontRight;        this.backLeft = backLeft;        this.backRight = backRight;        this.targetAngle = 0;    }    public void resetGyro(double offsetAngle){        Sensors.gyro.setDatum(Sensors.gyro.getIMUAngle() + offsetAngle);        targetAngle = offsetAngle;        releaseAngle = Sensors.gyro.getRawAngle();    }    @RequiresApi(api = Build.VERSION_CODES.N)    public double closestTarget(double targetAngle){        double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + Sensors.gyro.getRawAngle()) * 1e6), Math.round(360.000 * 1e6)) / 1e6;        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);        return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? Sensors.gyro.getRawAngle() - simpleTargetDelta : Sensors.gyro.getRawAngle() - alternateTargetDelta;    }            //mostly depreciated    public void setPower(double drive, double strafe, double turn, double power){        this.drive = Range.clip(drive, -1, 1);        this.strafe = Range.clip(strafe, -1, 1);        this.turn = Range.clip(turn, -1, 1);        this.power = Range.clip(power, 0 , 1);                if(this.power <= 0){            this.power = .05;        }            autoDrive = this.drive;        autoStrafe = this.strafe;        autoTurn = this.turn;        autoPower = this.power;                double flPower = (this.drive - this.strafe - this.turn) * this.power;        double frPower = (this.drive + this.strafe + this.turn) * this.power;        double blPower = (this.drive + this.strafe - this.turn) * this.power;        double brPower = (this.drive - this.strafe + this.turn) * this.power;        double maxPower = abs(max(max(abs(flPower), abs(frPower)), max(abs(blPower), abs(brPower))));                if (maxPower > 1) {            flPower = flPower / maxPower;            frPower = frPower / maxPower;            blPower = blPower / maxPower;            brPower = brPower / maxPower;        }        if(maxPower < .05 && maxPower > -.05) {            flPower = 0;            frPower = 0;            blPower = 0;            brPower = 0;        }        this.frontLeft.setPower(flPower);        this.frontRight.setPower(frPower);        this.backLeft.setPower(blPower);        this.backRight.setPower(brPower);    }            public void setPowerTele(double drive, double strafe, double turn, double power){        double inputTurn;                long currentTime = System.currentTimeMillis();        long deltaMili = currentTime - angleTimeRing.getValue(currentTime);        double deltaSeconds = deltaMili / 1000.0;                double currentAngle = Sensors.gyro.getRawAngle();        double deltaAngle = currentAngle - angleRing.getValue(currentAngle);        double rateOfChange = deltaAngle/deltaSeconds;                if(turn!= 0){            inputTurn = turn;            releaseAngle = currentAngle;                        if(rateOfChange != 0) {                adjRateOfChange = Math.pow(rateOfChange, 2) * (rateOfChange / abs(rateOfChange));            }else{                adjRateOfChange = 0;            }        }else{            double targetAngle = releaseAngle + .5 * .0035 * adjRateOfChange;            hmmm = targetAngle;            inputTurn = telePID.update( targetAngle - currentAngle);        }        setPower(drive, strafe, inputTurn, power);    }                public void setPowerAuto(double drive, double strafe, double targetAngle, double power){        this.targetAngle = targetAngle;        turn = autoPID.update(this.targetAngle - Sensors.gyro.getRawAngle());        setPower(drive, strafe, turn, power);    }        public void setPowerAuto(double drive, double strafe, double targetAngle){        setPowerAuto(drive, strafe, targetAngle,1.0);    }        public void setPowerVision(double drive, double strafe, double targetAngle, double power){        this.targetAngle = targetAngle;        turn = visionPID.update(this.targetAngle - Sensors.gyro.getRawAngle());        setPower(drive, strafe, turn, power);    }        public void setPowerVision(double drive, double strafe, double targetAngle){        setPowerAuto(drive, strafe, targetAngle,1.0);    }                    @RequiresApi(api = Build.VERSION_CODES.N)    public void strafe(double distance, double heading, double strafeAngle, double targetPower, double startPower, double endPower){            distance = abs((distance) / 0.0207);        startPower = abs(startPower);        targetPower = abs(targetPower);        endPower = abs(endPower);            double deceleratePower;        double accelRate = 0.0013;            if (isStrafeComplete){            resetWithoutEncoders();        }                double currentAngle = Sensors.gyro.getRawAngle();                double drive = Math.cos((strafeAngle - currentAngle) * PI / 180);        double strafe = Math.sin((strafeAngle - currentAngle) * PI / 180);                double currentDistance = adjustedTicks();                if(distance != 0){            double remainingDistance = distance - currentDistance;            deceleratePower = Math.sqrt(accelRate * (remainingDistance + 1.0/accelRate * Math.pow(endPower, 2))) + .05;            isStrafeComplete = currentDistance >= distance;        }else{            deceleratePower = 1;            isStrafeComplete = false;        }                double acceleratePower = Math.sqrt(accelRate * (currentDistance + 1.0/accelRate * Math.pow(startPower, 2))) + .05;        double currentPower = min(min(acceleratePower, deceleratePower), targetPower);                currentInches = (0.0207 * currentDistance);                setPowerAuto(drive, strafe, closestTarget(heading), currentPower);            }    public double adjustedTicks() {        double measuredTicks = ((abs(frontRight.getCurrentPosition()) + abs(frontLeft.getCurrentPosition()) + abs(backRight.getCurrentPosition()) + abs(backLeft.getCurrentPosition())) / 4.0);        double angleAdjustment = (abs(abs(drive) - abs(strafe)) - 1) * -1;        strafeDrive = power;        return Math.sqrt(Math.pow(measuredTicks, 2) + Math.pow(measuredTicks * angleAdjustment, 2));            }                @RequiresApi(api = Build.VERSION_CODES.N)    public void turn(double targetAngle, double targetPower, double startPower){        targetPower = abs(targetPower);        double accelRate = 0.03;        double currentAngle = Sensors.gyro.getRawAngle();                if(isTurnComplete){            startAngle = Sensors.gyro.getRawAngle();            resetWithEncoders();        }                double acceleratePower = Math.sqrt(accelRate * (abs(currentAngle - startAngle)+ 1/accelRate * Math.pow(startPower, 2))) + .05;        double currentPower = min(acceleratePower, targetPower);                if(startAngle > closestTarget(targetAngle)){            isTurnComplete = currentAngle < closestTarget(targetAngle) + 2.5;        }else{            isTurnComplete = currentAngle > closestTarget(targetAngle) - 2.5;        }                isStrafeComplete = isTurnComplete;                setPowerAuto(0,0, closestTarget(targetAngle), currentPower);    }                public void resetWithoutEncoders(){        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    }        public void resetWithEncoders(){        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);    }        public double getRPM(){        double retVal;                long currentTime = System.currentTimeMillis();        long deltaMili = currentTime - timeRing.getValue(currentTime);        double deltaMinutes = deltaMili / 60000.0;                double currentPositionFL = frontLeft.getCurrentPosition();        double currentPositionFR = frontRight.getCurrentPosition();        double currentPositionBL = backLeft.getCurrentPosition();        double currentPositionBR = backRight.getCurrentPosition();                double deltaRotationsFL = abs(currentPositionFL - positionRingFL.getValue(currentPositionFL)) / 537.6;        double deltaRotationsFR = abs(currentPositionFR - positionRingFR.getValue(currentPositionFR)) / 537.6;        double deltaRotationsBL = abs(currentPositionBL - positionRingBL.getValue(currentPositionBL)) / 537.6;        double deltaRotationsBR = abs(currentPositionBR - positionRingBR.getValue(currentPositionBR)) / 537.6;                retVal = ((deltaRotationsFL + deltaRotationsFR + deltaRotationsBL + deltaRotationsBR) / 4.0) / deltaMinutes;                return retVal;    }        @RequiresApi(api = Build.VERSION_CODES.N)    public void driveState(double drive, double strafe, double turn, double power){        power = (((power + 1) / -2) + 1.5);        switch ( currentDriveState){                    case STATE_FULL_CONTROL:                setPowerTele(drive, strafe, turn, power);                break;                    case STATE_NORTH:                if(turn != 0) { newState(DriveState.STATE_FULL_CONTROL); break; }                setPowerAuto(drive, strafe, closestTarget(0), power);                break;                    case STATE_EAST:                if(turn != 0) { newState(DriveState.STATE_FULL_CONTROL); break; }                setPowerAuto(drive, strafe, closestTarget(270), power);                break;                    case STATE_SOUTH:                if(turn != 0) { newState(DriveState.STATE_FULL_CONTROL); break; }                setPowerAuto(drive, strafe, closestTarget(180), power);                break;                    case STATE_WEST:                if(turn != 0) { newState(DriveState.STATE_FULL_CONTROL); break; }                setPowerAuto(drive, strafe, closestTarget(90), power);                break;                case STATE_ADJUSTMENT:                if(turn != 0) { newState(DriveState.STATE_FULL_CONTROL); break; }                setPowerAuto(drive, strafe, getAdjustmentAngle(), power);                break;            case STATE_TOWER_AIM:                if(turn != 0) { newState(DriveState.STATE_FULL_CONTROL); break; }                if (Sensors.frontCamera.isTowerFound()) setPowerVision(drive, strafe, closestTarget(getTowerAimAngle()), power);                else setPowerAuto(drive, strafe, closestTarget(90));                                break;                            case STATE_POWER_SHOT_AIM:                if(turn != 0) { newState(DriveState.STATE_FULL_CONTROL); break; }                if (Sensors.frontCamera.arePSFound()) setPowerVision(drive, strafe, closestTarget(getPSAngle()), power);                else setPowerAuto(drive, strafe, closestTarget(90));        }    }        @RequiresApi(api = Build.VERSION_CODES.N)    //if the boolean is true, it sets the targetAngle for the PID system to the closest coterminal angle to the input's respective angle (North 0, East 270, South 180, West 90) with priority being in that order    public void cardinalState(boolean north, boolean east, boolean south, boolean west){        if(north) { newState(DriveState.STATE_NORTH); }        if(east) { newState(DriveState.STATE_EAST); }        if(west) { newState(DriveState.STATE_WEST); }        if(south) { newState(DriveState.STATE_SOUTH); }    }    public void autoAimState(boolean autoAim, boolean autoAimToggle, boolean adjRight, boolean adjLeft, double adjDeltaAngle){        if(autoAimToggle && Shooter.currentShooterState == Shooter.ShooterState.STATE_TOP_GOAL){            Sensors.frontCamera.setPipeline(Sensors.frontCamera.towerAimPipeline);            if(autoAim || Shooter.shooterJustOn) { newState(DriveState.STATE_TOWER_AIM); }            setTowerAimAngle(Sensors.gyro.getRawAngle() + Sensors.frontCamera.towerAimError());        }                if(autoAimToggle && Shooter.currentShooterState == Shooter.ShooterState.STATE_POWER_SHOT) {            Sensors.frontCamera.setPipeline(Sensors.frontCamera.PSAimPipeline);            if (autoAim || Shooter.shooterJustOn || Shooter.feederJustOn || adjRight || adjLeft) {                newState(DriveState.STATE_POWER_SHOT_AIM);                                if(Shooter.shooterJustOn) powerShotState = 0;                if (Shooter.feederJustOn || adjRight) powerShotState++;                if (adjLeft) powerShotState--;                powerShotState = powerShotState % 3;                    switch (powerShotState) {                    case 0:                        setPSAngle(Sensors.gyro.getRawAngle() + Sensors.frontCamera.leftPSAimError());                            break;                            case 1:                        setPSAngle(Sensors.gyro.getRawAngle() + Sensors.frontCamera.centerPSAimError());                        break;                            case 2:                        setPSAngle(Sensors.gyro.getRawAngle() + Sensors.frontCamera.rightPSAimError());                        break;                }            }        }else{            if(adjRight) { newState(DriveState.STATE_ADJUSTMENT); setAdjustmentAngle(Sensors.gyro.getRawAngle() - adjDeltaAngle); }            if(adjLeft) { newState(DriveState.STATE_ADJUSTMENT); setAdjustmentAngle(Sensors.gyro.getRawAngle() + adjDeltaAngle); }        }    }        public double getAdjustmentAngle(){ return adjustmentAngle; }        public void setAdjustmentAngle(double adjustmentAngle){ this.adjustmentAngle = adjustmentAngle; }    public double getTowerAimAngle(){ return towerAimAngle; }        public double getPSAngle(){ return PSAngle; }        public void setPSAngle(double PSAngle){ this.PSAngle = PSAngle; }    public void setTowerAimAngle(double towerAimAngle){        this.towerAimAngle = towerAimAngle - 3;    }        private void newState(DriveState newState) {        currentDriveState = newState;    }                private enum DriveState {        STATE_FULL_CONTROL,        STATE_NORTH,        STATE_EAST,        STATE_SOUTH,        STATE_WEST,        STATE_ADJUSTMENT,        STATE_TOWER_AIM,        STATE_POWER_SHOT_AIM    }    }