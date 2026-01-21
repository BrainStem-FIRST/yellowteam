package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterLookup.dists;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterLookup.newDists;

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
import org.firstinspires.ftc.teamcode.opmode.testing.EfficiencyCoefficientTester;
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
        public double maxErrorThresholdNear = 750, maxErrorThresholdFar = 90;
        public double shotVelDropThreshold = 50;
        public double noiseVariance = 40;
        public boolean printShootInfo = false;
        public int startingShooterSpeedAdjustment = 0;
        public double minPower = -0.15, maxPower = 0.99;
        public double shotRecoveryPower = 0.99, shotRecoveryError = 40;
    }
    public static class EnergyLossParams {
        public double distanceM = 0, distanceB = 0.392;
//        public double m = -0.002;
//        public double b = 0.72;

        public double empiricalGoalHeightInches = 38;
        public double empiricalExitPosDistFromGoalNegOffset = 0;
        public boolean testingEfficiency = false;
        public double testingEfficiencyCoef = 1;
    }
    public static class HoodParams {
        public double downPWM = 900, upPWM = 2065;
    }
    public static class TestingParams {
        public boolean testing = false;
        public double testingExitSpeedMetersPerSec = 5;
        public double testingVel = 1500;
        public double testingExitAngleRad = 1.0472;
    }

    public static ShooterParams shooterParams = new ShooterParams();
    public static HoodParams hoodParams = new HoodParams();
    public static EnergyLossParams energyLossParams = new EnergyLossParams();
    public static TestingParams testingParams = new TestingParams();

    public enum ShooterState {
        OFF, UPDATE
    }
    public DcMotorEx shooterMotorLow;
    public DcMotorEx shooterMotorHigh;
    public ServoImplEx hoodLeftServo;
    public ServoImplEx hoodRightServo;
    public ShooterState shooterState;

    public final PIDController shooterPID;
    private double ballExitAngleRad;
    private double nearVelocityAdjustment, farVelocityAdjustment;
    public Vector2d ballExitPosition, prevExitPos;
    private double targetMotorVelTps, avgMotorVelTps, prevVelTps;
    public double curExitVelMps, targetExitVelMps;
    private double efficiencyCoef;
    private int ballsShot;
    public double lastMax, lastMin, lastDecel, velDropTime;
    private final ArrayList<Double> allVelDrops, allPostShotVels, allLastDecels, allVelDropTimes;
    private double mSOfLastMax;
    private boolean increasing, wasPrevIncreasing;
    private final ShooterLookup shooterLookup;
    private double ballExitPosInchesFromGoal;
    public boolean isNear;
    public Vector2d robotVelAtExitPosInchesPerSec, robotDispAtExitPosInches;
    public double robotSpeedAtExitPosInchesPerSec;
    private double curTimeMs, dt;

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
        nearVelocityAdjustment = shooterParams.startingShooterSpeedAdjustment;
        farVelocityAdjustment = shooterParams.startingShooterSpeedAdjustment;

        allVelDrops = new ArrayList<>();
        allPostShotVels = new ArrayList<>();
        allLastDecels = new ArrayList<>();
        allVelDropTimes = new ArrayList<>();

        shooterLookup = new ShooterLookup();
        curTimeMs = System.currentTimeMillis();
        if(robot != null)
            ballExitPosition = ShootingMath.calculateExitPositionInches(robot.drive.localizer.getPose(), robot.turret.getTurretEncoder(), ballExitAngleRad);
    }

    public double getBallExitAngleRad() {
        return ballExitAngleRad;
    }
    public void setShooterPower(double power) {
        shooterMotorHigh.setPower(power);
        shooterMotorLow.setPower(power);
    }
    public void setShooterVelocityPID(double targetVelocityTicksPerSec, double currentShooterVelocity) {
        if (isNear)
            shooterPID.setTarget(targetVelocityTicksPerSec + nearVelocityAdjustment);
        else
            shooterPID.setTarget(targetVelocityTicksPerSec + farVelocityAdjustment);

        double pidOutput = -shooterPID.update(currentShooterVelocity);
        double feedForward = shooterParams.kF * targetVelocityTicksPerSec;
        double totalPower = pidOutput + feedForward;

        totalPower = Range.clip(totalPower, shooterParams.minPower, shooterParams.maxPower);
        double error = targetVelocityTicksPerSec - currentShooterVelocity;
        boolean recovering = false;
        if(error > shooterParams.shotRecoveryError) {
            totalPower = shooterParams.shotRecoveryPower;
            recovering = true;
        }
        if (shooterParams.printShootInfo) {
            telemetry.addData("recovering", recovering);
            telemetry.addData("pid output", pidOutput);
            telemetry.addData("total power", totalPower);
        }

        setShooterPower(totalPower);
    }
    public void updateShooterSystemPhysics(double dist) {
        dist = Range.clip(dist, newDists[0] + 0.01, newDists[newDists.length-1] - 0.01);

        double ballExitSpeedMps, ballExitAngleRad;
        if (testingParams.testing) {
            ballExitSpeedMps = testingParams.testingExitSpeedMetersPerSec;
            ballExitAngleRad = testingParams.testingExitAngleRad;
        }
        else {
            ballExitSpeedMps = shooterLookup.lookupVelocityMetersPerSec(dist);
            ballExitAngleRad = ShootingMath.calculateBallExitAngleRad(false, isNear, dist, ballExitSpeedMps);
        }
        // vel target = vel actual * getEfficiency(vel target)
        // vel actual = vel target / getEfficiency(vel target)
        double targetExitVelTicksPerSec = ShootingMath.exitMpsToMotorTicksPerSec(ballExitSpeedMps, 1);
        double efficiencyCoefficient = -0.002 * Math.toDegrees(ballExitAngleRad) + 0.72;
        double actualVelTicksPerSec = targetExitVelTicksPerSec / efficiencyCoefficient;

        setShooterVelocityPID(actualVelTicksPerSec, avgMotorVelTps);;
    }
    public void updateShooterSystemLookupTable(double dist) {
        dist = Range.clip(dist, dists[0] + 0.01, dists[dists.length-1] - 0.01);
        if(testingParams.testing) {
            targetMotorVelTps = testingParams.testingVel;
            ballExitAngleRad = testingParams.testingExitAngleRad;
        }
        else {
//            targetMotorVelTps = shooterLookup.lookupVelocityTicksPerSec(dist);
            ballExitAngleRad = shooterLookup.lookupExitAngleRad(dist);
        }
        setShooterVelocityPID(targetMotorVelTps, avgMotorVelTps);
        double hoodServoPos = ShootingMath.calculateHoodServoPosition(ballExitAngleRad);
        setHoodPosition(hoodServoPos);
    }

    public void setHoodPosition(double position) {
        hoodLeftServo.setPosition(position);
        hoodRightServo.setPosition(position);
    }
    public void updateProperties() {
        avgMotorVelTps = getAvgMotorVelocity();
        efficiencyCoef = energyLossParams.testingEfficiency ? energyLossParams.testingEfficiencyCoef : energyLossParams.distanceM * ballExitPosInchesFromGoal + energyLossParams.distanceB;
        curExitVelMps = ShootingMath.ticksPerSecToExitSpeedMps(avgMotorVelTps, efficiencyCoef);
        int turretEncoder = robot.turret.getTurretEncoder();

        Pose2d robotPose = robot.drive.localizer.getPose();
        prevExitPos = new Vector2d(ballExitPosition.x, ballExitPosition.y);
        ballExitPosition = ShootingMath.calculateExitPositionInches(robotPose, turretEncoder, ballExitAngleRad);
        double deltaX = robot.turret.targetPose.position.x - ballExitPosition.x;
        double deltaY = robot.turret.targetPose.position.y - ballExitPosition.y;
        ballExitPosInchesFromGoal = Math.hypot(deltaX, deltaY);

        targetMotorVelTps = shooterLookup.lookupVelocityTicksPerSec(ballExitPosInchesFromGoal);
        targetExitVelMps = ShootingMath.ticksPerSecToExitSpeedMps(targetMotorVelTps, efficiencyCoef);

        double prevTimeMs = curTimeMs;
        curTimeMs = System.currentTimeMillis();
        robotDispAtExitPosInches = ballExitPosition.minus(prevExitPos);
        dt = (curTimeMs - prevTimeMs) / 1000;
        robotVelAtExitPosInchesPerSec = robotDispAtExitPosInches.div(dt);
        this.robotSpeedAtExitPosInchesPerSec = Math.hypot(robotVelAtExitPosInchesPerSec.x, robotVelAtExitPosInchesPerSec.y);


        if(ShootingMath.enableRelativeVelocity) {
            double relativeShooterVelMps = Math.hypot(ShootingMath.relativeBallExitVelocityMps.x, ShootingMath.relativeBallExitVelocityMps.y);
            targetMotorVelTps = ShootingMath.exitMpsToMotorTicksPerSec(relativeShooterVelMps, efficiencyCoef);
        }

        isNear = robotPose.position.x < 24;
    }

    @Override
    public void update(){
        switch (shooterState) {
            case OFF:
                setShooterPower(0);
                break;

            case UPDATE:
                updateShooterSystemLookupTable(ballExitPosInchesFromGoal);
//                updateShooterSystemPhysics(ballExitPosInchesFromGoal);
                break;
        }
        updateBallShotTracking();
    }
    public void updateBallShotTracking() {
        double dif = avgMotorVelTps - prevVelTps;
        if(dif > 0)
            increasing = true;
        else if(dif == 0)
            increasing = wasPrevIncreasing;
        else
            increasing = false;

        if(increasing && !wasPrevIncreasing) {  // means relative min detected
            lastMin = prevVelTps;
            double velDrop = lastMax - lastMin;
            velDropTime = (System.currentTimeMillis() - mSOfLastMax) / 1000;
            lastDecel = velDrop / velDropTime;
            if(velDrop >= shooterParams.shotVelDropThreshold
            || shooterPID.getTarget() - lastMin >= shooterParams.noiseVariance) {
                ballsShot++;
                allVelDrops.add(velDrop);
                allPostShotVels.add(lastMin);
                allLastDecels.add(lastDecel);
                allVelDropTimes.add(velDropTime);
            }
        }
        if(wasPrevIncreasing && !increasing) { // means relative max detected
            lastMax = prevVelTps;
            mSOfLastMax = System.currentTimeMillis();
        }

        prevVelTps = avgMotorVelTps;
        wasPrevIncreasing = increasing;

        if(robot.collection.getClutchState() != Collection.ClutchState.ENGAGED) {
            ballsShot = 0;
            allVelDrops.clear();
            allPostShotVels.clear();
            allLastDecels.clear();
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
        telemetry.addData("  target vel", targetMotorVelTps);
        telemetry.addData("  avg motor vel TPS", avgMotorVelTps);
        telemetry.addData("  pid target vel", shooterPID.getTarget());
        telemetry.addData("  shooter high power", shooterMotorHigh.getPower());
//        telemetry.addData("  efficiency coef", efficiencyCoef);
        telemetry.addData("  exit vel MPS using efficiencyCoef", curExitVelMps);
        double changeInYMeters = energyLossParams.empiricalGoalHeightInches * 0.0254 - ShootingMath.calculateExactExitHeightMeters(ballExitAngleRad);
        telemetry.addData("change in y meters", changeInYMeters);
        telemetry.addData("  ACTUAL exit vel MPS", EfficiencyCoefficientTester.calculateTargetExitVelocity((ballExitPosInchesFromGoal - energyLossParams.empiricalExitPosDistFromGoalNegOffset) * 0.0254, changeInYMeters, ballExitAngleRad));

        telemetry.addLine();
        telemetry.addData("dt", dt);
        telemetry.addData("exit disp in one frame", MathUtils.format3(Math.hypot(robotDispAtExitPosInches.x, robotDispAtExitPosInches.y)));
        telemetry.addData("relative exit vel mps", MathUtils.formatVec(ShootingMath.relativeBallExitVelocityMps));
        telemetry.addData("robot vel at exit pos in/s", MathUtils.formatVec(robotVelAtExitPosInchesPerSec));
        telemetry.addData("robot speed at exit pos in/s", robotSpeedAtExitPosInchesPerSec);
//        telemetry.addData("shooter velocity error", targetMotorVelTps - avgMotorVelTps);
//        telemetry.addData("  ball vel drops", Arrays.toString(velDrops.toArray()));
//        telemetry.addData("  post shot vels", Arrays.toString(postShotVels.toArray()));
//        telemetry.addData("  last max", lastMax);
//        telemetry.addData("  last min", lastMin);
//        telemetry.addData("increasing", increasing ? 50 : -50);
//        telemetry.addData("balls shot", ballsShot);
//        telemetry.addData("target - lastMin", targetMotorVelTps - lastMin);
//        telemetry.addData("noise variance", shooterParams.noiseVariance);
//        telemetry.addData("shot vel threshold", shooterParams.shotVelDropThreshold);
//        telemetry.addData("last extrema dif", MathUtils.format3(lastMax - lastMin));
//        telemetry.addData("all velDropTimes", Arrays.toString(allVelDropTimes.toArray()));
//        telemetry.addData("vel drop time", velDropTime);
//        telemetry.addData("all decels", Arrays.toString(allLastDecels.toArray()));
//        telemetry.addData("last decel", lastDecel);
//        telemetry.addData("shooter error", avgMotorVel - shooterPID.getTarget());

        telemetry.addLine();
        telemetry.addLine("HOOD------");
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

    public void changeVelocityAdjustment(double amount) {
        if (isNear)
            nearVelocityAdjustment += amount;
        farVelocityAdjustment += amount;
    }
}
