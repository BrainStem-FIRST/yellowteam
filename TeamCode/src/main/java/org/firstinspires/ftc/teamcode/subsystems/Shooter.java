package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.shootingRecording.AutomaticShooterSpeedRecorder;
import org.firstinspires.ftc.teamcode.utils.shootingRecording.ManualShooterSpeedRecorder;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;
import org.firstinspires.ftc.teamcode.utils.shootingRecording.ShotData;

import java.util.Collections;
import java.util.Set;

@Config
public class Shooter extends Component {
    public static class ShooterParams {
        public double kP = 0.005;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.00048;
        public double maxErrorThresholdNear = 75, maxErrorThresholdFar = 60;
        public double shotVelDropThreshold = 70;
    }
    public static class HoodParams {
        public double downPWM = 900, upPWM = 2065, moveThresholdAngleDeg = 0.5;
    }

    public static ShooterParams SHOOTER_PARAMS = new ShooterParams();
    public static HoodParams HOOD_PARAMS = new HoodParams();
    public static boolean printShootInfo = false;
    public static boolean fixHoodToIdealVelocity = true;
    public static int startingShooterSpeedAdjustment = -20;

    public enum ShooterState {
        OFF, UPDATE, REVERSE_FULL
    }
    public DcMotorEx shooterMotorLow;
    public DcMotorEx shooterMotorHigh;
    public ServoImplEx hoodLeftServo;
    public ServoImplEx hoodRightServo;
    public ShooterState shooterState;

    public final PIDController shooterPID;
    private double ballExitAngleRad;
    public double adjustment;
    private final ElapsedTime recordTimer;
    public Vector2d ballExitPosition;
    private int ballsShot;
    public boolean isNear;
    private double prevVel, lastMax, lastMin;
    private boolean wasPrevIncreasing;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        shooterMotorLow = hardwareMap.get(DcMotorEx.class, "lowShoot");
        shooterMotorLow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLow.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorLow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotorHigh = hardwareMap.get(DcMotorEx.class, "highShoot");
        shooterMotorHigh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorHigh.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorHigh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hoodLeftServo = hardwareMap.get(ServoImplEx.class, "hoodLeft");
        hoodLeftServo.setPwmRange(new PwmControl.PwmRange(HOOD_PARAMS.downPWM, HOOD_PARAMS.upPWM));

        hoodRightServo = hardwareMap.get(ServoImplEx.class, "hoodRight");
        hoodRightServo.setPwmRange(new PwmControl.PwmRange(HOOD_PARAMS.downPWM, HOOD_PARAMS.upPWM));

        shooterPID = new PIDController(SHOOTER_PARAMS.kP, SHOOTER_PARAMS.kI, SHOOTER_PARAMS.kD);

        shooterState = ShooterState.OFF;
        recordTimer = new ElapsedTime();
        recordTimer.reset();
        lastMax = 0;
        lastMin = Double.MAX_VALUE;
        ballsShot = 0;
        adjustment = startingShooterSpeedAdjustment;

    }

    public double getBallExitAngleRad() {
        return ballExitAngleRad;
    }
    public void setShooterPower(double power) {
        shooterMotorHigh.setPower(power);
        shooterMotorLow.setPower(power);
    }
    public void setShooterVelocityPID(double targetVelocityTicksPerSec) {
        shooterPID.setTarget(targetVelocityTicksPerSec + adjustment);

        double currentVelocity = getAvgMotorVelocity();
        double pidOutput = -shooterPID.update(currentVelocity);
        double feedForward = SHOOTER_PARAMS.kF * targetVelocityTicksPerSec;
        double totalPower = pidOutput + feedForward;

        totalPower = Math.abs(Range.clip(totalPower, -0.99, 0.99));
        if (printShootInfo) {
            telemetry.addData("pid output", pidOutput);
            telemetry.addData("total power", totalPower);
        }

        setShooterPower(totalPower);
    }
    public void updateShooterSystem(Vector2d ballExitPosition, Pose2d targetPose, boolean powerShooterAndHood) {
        double deltaX = targetPose.position.x - ballExitPosition.x;
        double deltaY = targetPose.position.y - ballExitPosition.y;
        double ballExitPosInchesFromGoal = Math.hypot(deltaX, deltaY);

        isNear = ballExitPosInchesFromGoal < ShootingMath.shooterSystemParams.closeToFarZoneThresholdInches;
        boolean shootHighArc = ballExitPosInchesFromGoal < ShootingMath.shooterSystemParams.highToLowArcThresholdInches;

        // update FLYWHEEL
        OdoInfo mostRecentVelocity = robot.drive.pinpoint().getMostRecentVelocity();
        double targetShooterVelocityTicksPerSec = ShootingMath.calculateShooterMotorSpeedTicksPerSec(telemetry, targetPose, isNear, shootHighArc, ballExitPosInchesFromGoal, ballExitPosition, mostRecentVelocity);

        if (powerShooterAndHood)
            setShooterVelocityPID(targetShooterVelocityTicksPerSec);

        // update HOOD
        // offset the hood's distance when close
        double hoodBallExitPosInchesFromGoal = ballExitPosInchesFromGoal;
        if (shootHighArc)
            hoodBallExitPosInchesFromGoal += ShootingMath.shooterSystemParams.highArcGoalOffsetInches;
        else
            hoodBallExitPosInchesFromGoal += isNear ? ShootingMath.shooterSystemParams.lowArcNearGoalOffsetInches : ShootingMath.shooterSystemParams.lowArcFarGoalOffsetInches;

        double shooterSpeed = fixHoodToIdealVelocity ? targetShooterVelocityTicksPerSec : getAvgMotorVelocity();
        double currentBallExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(shooterSpeed);
        double oldBallExitAngleRad = ballExitAngleRad;
        ballExitAngleRad = ShootingMath.calculateBallExitAngleRad(targetPose, ballExitPosition, shootHighArc, isNear, hoodBallExitPosInchesFromGoal, currentBallExitSpeedMps, mostRecentVelocity, telemetry);
        double hoodServoPos = ShootingMath.calculateHoodServoPosition(ballExitAngleRad, printShootInfo ? telemetry : null);
        if (powerShooterAndHood && ballExitAngleRad != -1 && Math.abs(ballExitAngleRad - oldBallExitAngleRad) >= Math.toRadians(HOOD_PARAMS.moveThresholdAngleDeg))
            setHoodPosition(hoodServoPos);

        if (printShootInfo) {
            telemetry.addData("use high arc", shootHighArc);
            telemetry.addData("in near zone", isNear);
            telemetry.addData("actual distance from goal inches", ballExitPosInchesFromGoal);
            telemetry.addData("distance from goal for hood", hoodBallExitPosInchesFromGoal);
            telemetry.addData("target motor speed", targetShooterVelocityTicksPerSec);
            telemetry.addData("actual motor speed", getAvgMotorVelocity());
            telemetry.addData("exit speed mps", currentBallExitSpeedMps);
            telemetry.addData("ball exit position inches", ballExitPosition.x + ", " + ballExitPosition.y);
            telemetry.addData("ball exit angle deg", Math.toDegrees(ballExitAngleRad));
            if (ballExitAngleRad == -1)
                telemetry.addLine("NO SOLUTION FOR HOOD=====================");
        }
    }

    public void setHoodPosition(double position) {
        hoodLeftServo.setPosition(position);
        hoodRightServo.setPosition(position);
    }

    @Override
    public void update(){
        double vel = getAvgMotorVelocity();
        double dif = vel - prevVel;
        boolean increasing;
        if(dif > 0)
            increasing = true;
        else if(dif == 0)
            increasing = wasPrevIncreasing;
        else
            increasing = false;

        switch (shooterState) {
            case OFF:
                setShooterPower(0);
                ballExitPosition = ShootingMath.calculateExitPositionInches(robot.drive.localizer.getPose(), robot.turret.getTurretEncoder(), ballExitAngleRad);
                updateShooterSystem(ballExitPosition, robot.turret.targetPose, false);
                break;

            case UPDATE:
                if(increasing && !wasPrevIncreasing) { // means relative min detected
                    lastMin = prevVel;
                    if(lastMax - lastMin > SHOOTER_PARAMS.shotVelDropThreshold)
                        ballsShot++;
                }
                if(wasPrevIncreasing && !increasing) // means relative max detected
                    lastMax = prevVel;

                ballExitPosition = ShootingMath.calculateExitPositionInches(robot.drive.localizer.getPose(), robot.turret.getTurretEncoder(), ballExitAngleRad);
                updateShooterSystem(ballExitPosition, robot.turret.targetPose, true);
                break;
            case REVERSE_FULL:
                setShooterPower(-0.99);
                break;
        }
        prevVel = vel;
        wasPrevIncreasing = increasing;

        updateManualShooterTracking();
    }

    public double getVelHigh() {
        return Math.abs(shooterMotorHigh.getVelocity());
    }
    public double getVelLow() {
        return Math.abs(shooterMotorLow.getVelocity());
    }

    @Override
    public void printInfo() {
        telemetry.addLine("SHOOTER------");
        telemetry.addData("  state", shooterState);
        telemetry.addData("  balls shot", ballsShot);
        telemetry.addData("  avg motor vel", getAvgMotorVelocity());
        telemetry.addData("  last max", lastMax);
        telemetry.addData("  last min", lastMin);
        telemetry.addData("shooter error", getAvgMotorVelocity() - shooterPID.getTarget());
        telemetry.addData("  pid target vel", shooterPID.getTarget());
        telemetry.addData("  power high", shooterMotorHigh.getPower());
        telemetry.addData("  power low", shooterMotorLow.getPower());

        telemetry.addLine();
        telemetry.addLine("HOOD------");
        telemetry.addData("  ball exit angle rad", ballExitAngleRad);
        telemetry.addData("  left servo pos", hoodLeftServo.getPosition());
        telemetry.addData("  right servo pos", hoodRightServo.getPosition());
    }

    public double getAvgMotorVelocity() {
        return (getVelHigh() + getVelLow()) * 0.5;
    }

    private boolean trackData = false;
    private int num;
    private double lastTime;
    private void updateManualShooterTracking() {
        if (trackData) {
            if (num >= ManualShooterSpeedRecorder.recordAmountForEachShot)
                throw new IllegalStateException("tracking data for too long - " + num + " is out of range for " + ManualShooterSpeedRecorder.recordAmountForEachShot + " data slots");

            if (recordTimer.milliseconds() - lastTime > ManualShooterSpeedRecorder.recordIntervalMs
                    && num < ManualShooterSpeedRecorder.recordAmountForEachShot
                    && ManualShooterSpeedRecorder.getCurrentShot() < ManualShooterSpeedRecorder.numShotsToRecord) {
                lastTime = recordTimer.milliseconds();
                ManualShooterSpeedRecorder.data[ManualShooterSpeedRecorder.getCurrentShot()][num][0] = recordTimer.seconds();
                ManualShooterSpeedRecorder.data[ManualShooterSpeedRecorder.getCurrentShot()][num][1] = shooterPID.getTarget();
                ManualShooterSpeedRecorder.data[ManualShooterSpeedRecorder.getCurrentShot()][num][2] = getAvgMotorVelocity();
                ManualShooterSpeedRecorder.data[ManualShooterSpeedRecorder.getCurrentShot()][num][3] = shooterMotorHigh.getPower();
                num++;
            }
        }
    }
    public boolean isTrackingData() {
        return trackData;
    }
    public void startManualTrackingData() {
        trackData = true;
        num = 0;
        lastTime = 0;
    }
    public void stopManualTrackingData() {
        trackData = false;
        ManualShooterSpeedRecorder.incrementCurrentShot();
        lastTime = 0;
        num = 0;
    }

    public Command automaticShooterTrackerCommand() {
        return new Command() {
            private final ElapsedTime timer = new ElapsedTime();
            @Override
            public void initialize() {
                AutomaticShooterSpeedRecorder.resetData();
                timer.reset();
            }
            @Override
            public void execute() {
                if (shooterState == ShooterState.UPDATE && robot.collection.getClutchState() == Collection.ClutchState.ENGAGED && robot.collection.getCollectionState() == Collection.CollectionState.INTAKE)
                    AutomaticShooterSpeedRecorder.addShotData(new ShotData(
                            timer.seconds(),
                            ballsShot,
                            getAvgMotorVelocity(),
                            shooterPID.getTarget() - adjustment, shooterPID.getTarget(),
                            shooterMotorHigh.getPower(),
                            Math.toDegrees(ballExitAngleRad),
                            robot.turret.getTurretEncoder(), robot.turret.targetEncoder));
            }

            @Override
            public Set<Subsystem> getRequirements() { return Collections.emptySet(); }
        };
    }
    public void setBallsShot(int n) {
        ballsShot = n;
    }
    public int getBallsShot() {
        return ballsShot;
    }
}
