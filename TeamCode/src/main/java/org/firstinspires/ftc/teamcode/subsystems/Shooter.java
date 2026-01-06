package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterLookup.dists;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;

import java.util.ArrayList;

@Config
public class Shooter extends Component {
    public static class ShooterParams {
        public double kP = 0.005;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.00045;
        public double maxErrorThresholdNear = 750, maxErrorThresholdFar = 100;
        public double shotVelDropThreshold = 40;
        public double noiseVariance = 40;
    }
    public static class HoodParams {
        public double downPWM = 900, upPWM = 2065;
    }
    public static class TestingParams {
        public boolean testing = false;
        public double testingVel = 1500;
        public double testingExitAngleRad = 1.0472;
    }

    public static ShooterParams shooterParams = new ShooterParams();
    public static HoodParams hoodParams = new HoodParams();
    public static TestingParams testingParams = new TestingParams();
    public static boolean printShootInfo = true;
    public static int startingShooterSpeedAdjustment = 0;

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
    public Vector2d ballExitPosition;
    private int ballsShot;
    private double targetMotorVel, avgMotorVel, prevVel, lastMax, lastMin;
    private final ArrayList<Double> velDrops;
    private final ArrayList<Double> postShotVels;
    private boolean increasing, wasPrevIncreasing;
    private final ShooterLookup shooterLookup;
    private double ballExitPosInchesFromGoal;
    public boolean isNear;

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
        hoodLeftServo.setPwmRange(new PwmControl.PwmRange(hoodParams.downPWM, hoodParams.upPWM));

        hoodRightServo = hardwareMap.get(ServoImplEx.class, "hoodRight");
        hoodRightServo.setPwmRange(new PwmControl.PwmRange(hoodParams.downPWM, hoodParams.upPWM));

        shooterPID = new PIDController(shooterParams.kP, shooterParams.kI, shooterParams.kD);

        shooterState = ShooterState.OFF;
        lastMax = 0;
        lastMin = Double.MAX_VALUE;
        ballsShot = 0;
        adjustment = startingShooterSpeedAdjustment;

        velDrops = new ArrayList<>();
        postShotVels = new ArrayList<>();

        shooterLookup = new ShooterLookup();
    }

    public double getBallExitAngleRad() {
        return ballExitAngleRad;
    }
    public void setShooterPower(double power) {
        shooterMotorHigh.setPower(power);
        shooterMotorLow.setPower(power);
    }
    public void setShooterVelocityPID(double targetVelocityTicksPerSec, double currentShooterVelocity) {
        shooterPID.setTarget(targetVelocityTicksPerSec + adjustment);

        double pidOutput = -shooterPID.update(currentShooterVelocity);
        double feedForward = shooterParams.kF * targetVelocityTicksPerSec;
        double totalPower = pidOutput + feedForward;

        totalPower = Math.abs(Range.clip(totalPower, -0.99, 0.99));
        if (printShootInfo) {
            telemetry.addData("pid output", pidOutput);
            telemetry.addData("total power", totalPower);
        }

        setShooterPower(totalPower);
    }
    public void updateShooterSystem(double dist) {
        dist = Range.clip(dist, dists[0] + 0.01, dists[dists.length-1] - 0.01);
        if(testingParams.testing) {
            targetMotorVel = testingParams.testingVel;
            ballExitAngleRad = testingParams.testingExitAngleRad;
        }
        else {
            targetMotorVel = shooterLookup.lookupVelocityTicksPerSec(dist);
            ballExitAngleRad = shooterLookup.lookupExitAngleRad(dist);
        }
        setShooterVelocityPID(targetMotorVel, avgMotorVel);
        double hoodServoPos = ShootingMath.calculateHoodServoPosition(ballExitAngleRad, printShootInfo ? telemetry : null);
        setHoodPosition(hoodServoPos);
    }

    public void setHoodPosition(double position) {
        hoodLeftServo.setPosition(position);
        hoodRightServo.setPosition(position);
    }

    @Override
    public void update(){
        avgMotorVel = getAvgMotorVelocity();
//        setHoodPosition(ShootingMath.calculateHoodServoPosition(HOOD_PARAMS.testingExitAngleRad, telemetry));
        telemetry.addData("shooter tangential vel m/s", ShootingMath.ticksPerSecToExitSpeedMps(avgMotorVel));

        int turretEncoder = robot.turret.getTurretEncoder();


        Pose2d robotPose = robot.drive.localizer.getPose();
        ballExitPosition = ShootingMath.calculateExitPositionInches(robotPose, turretEncoder, ballExitAngleRad);
        double deltaX = robot.turret.targetPose.position.x - ballExitPosition.x;
        double deltaY = robot.turret.targetPose.position.y - ballExitPosition.y;
        ballExitPosInchesFromGoal = Math.hypot(deltaX, deltaY);

        isNear = robotPose.position.x < 24;

                switch (shooterState) {
            case OFF:
                setShooterPower(0);
                break;

            case UPDATE:
                updateShooterSystem(ballExitPosInchesFromGoal);
                break;
            case REVERSE_FULL:
                setShooterPower(-0.99);
                break;
        }
        updateBallShotTracking();
//
//        updateManualShooterTracking();
    }
    public void updateBallShotTracking() {
        double dif = avgMotorVel - prevVel;
        if(dif > 0)
            increasing = true;
        else if(dif == 0)
            increasing = wasPrevIncreasing;
        else
            increasing = false;

        if(increasing && !wasPrevIncreasing) {  // means relative min detected
            lastMin = prevVel;
            double velDrop = lastMax - lastMin;
            if(velDrop >= shooterParams.shotVelDropThreshold
            || shooterPID.getTarget() - lastMin >= shooterParams.noiseVariance) {
                ballsShot++;
                velDrops.add(velDrop);
                postShotVels.add(lastMin);
            }
        }
        if(wasPrevIncreasing && !increasing) // means relative max detected
            lastMax = prevVel;

        prevVel = avgMotorVel;
        wasPrevIncreasing = increasing;

        if(robot.collection.getClutchState() != Collection.ClutchState.ENGAGED) {
            ballsShot = 0;
            velDrops.clear();
            postShotVels.clear();
        }
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
//        telemetry.addData("  state", shooterState);
//        telemetry.addData("isNear", isNear);
        telemetry.addData("  ball exit pos dist from goal", MathUtils.format3(ballExitPosInchesFromGoal));
        telemetry.addData("  lookup motor vel", targetMotorVel);
        telemetry.addData("  avg motor vel", avgMotorVel);
        telemetry.addData(" scaled motor vel", avgMotorVel / 1500 * 40);
        telemetry.addData("  pid target vel", shooterPID.getTarget());
        telemetry.addData("  shooter high power", shooterMotorHigh.getPower());
        telemetry.addData("shooter velocity error", targetMotorVel - avgMotorVel);
//        telemetry.addData("  ball vel drops", Arrays.toString(velDrops.toArray()));
//        telemetry.addData("  post shot vels", Arrays.toString(postShotVels.toArray()));
//        telemetry.addData("  last max", lastMax);
//        telemetry.addData("  last min", lastMin);
        telemetry.addData("increasing", increasing ? 50 : -50);
        telemetry.addData("  balls shot", ballsShot);
        telemetry.addData("dist from target", targetMotorVel - lastMin);
        telemetry.addData("noise variance", shooterParams.noiseVariance);
        telemetry.addData("shot vel threshold", shooterParams.shotVelDropThreshold);
        telemetry.addData("  last extrema dif", MathUtils.format3(lastMax - lastMin));
//        telemetry.addData("shooter error", avgMotorVel - shooterPID.getTarget());

        telemetry.addLine();
//        telemetry.addLine("HOOD------");
        telemetry.addData("  lookup hood exit angle rad", MathUtils.format3(ballExitAngleRad));
        telemetry.addData("  lookup hood exit angle deg", MathUtils.format3(Math.toDegrees(ballExitAngleRad)));
        telemetry.addData("  hood left servo pos", hoodLeftServo.getPosition());
//        telemetry.addData("  right servo pos", hoodRightServo.getPosition());
    }

    public double getAvgMotorVelocity() {
        return (getVelHigh() + getVelLow()) * 0.5;
    }
    public void setBallsShot(int n) {
        ballsShot = n;
    }
    public int getBallsShot() {
        return ballsShot;
    }
}
