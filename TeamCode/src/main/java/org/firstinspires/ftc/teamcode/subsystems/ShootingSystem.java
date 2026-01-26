package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterLookup.lookupDistsI;

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
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.math.Vector3d;
import org.firstinspires.ftc.teamcode.utils.misc.MotorCacher;
import org.firstinspires.ftc.teamcode.utils.misc.ServoImplCacher;

import java.util.Arrays;

@Config
public class ShootingSystem {
    public static class TestingParams {
        public boolean usingLookup = false;
        public boolean enableShootingWhileMoving = false;
        public boolean dynamicHood = true;
    }
    public static class GoalParams {
        public double nearRedX = -63, nearRedY = 63;
        public double midRedX = -65, midRedY = 65;
        public double farRedX = -66, farRedY = 65;
        public double nearBlueX = -66, nearBlueY = -65;
        public double midBlueX = -66, midBlueY = -65;
        public double farBlueX = -67, farBlueY = -63;
        public double nearHeight = 38, midHeight = 39, farHeight = 42;
        public double nearImpactAng = Math.toRadians(-30), midImpactAng = -Math.toRadians(25), farImpactAng = Math.toRadians(25);
        public double nearStateThreshold = 58;
    }
    public static GoalParams goalParams = new GoalParams();
    public static class HoodParams {
        public double downPWM = 900, upPWM = 2065;
        public double minExitAngRad = Math.toRadians(35), maxExitAngRad = Math.toRadians(85);
        public double resolution = 0.005;
    }
    public static class GeneralParams {
        public int lookAheadAvgNum = 5;
        public double rawLookAheadTime = 0.2; // time to look ahead for pose prediction
        public double shooterTau = 0.2;
        public int numApproximations = 4;
        // efficiency coef regression: y=-0.0766393x+0.446492
        public double efficiencyCoefM = -0.0766393, efficiencyCoefB = 0.446492;
        public double minEfficiencyCoef = 0.3327, maxEfficiencyCoef = 0.4000;
    }
    public static TestingParams testingParams = new TestingParams();
    public static HoodParams hoodParams = new HoodParams();
    public static GeneralParams generalParams = new GeneralParams();

    public enum Dist {
        NEAR, MID, FAR
    }
    public Dist distState;
    public boolean usingHighArc;
    public final Vector3d nearGoalPos, midGoalPos, farGoalPos;

    private final HardwareMap hardwareMap;
    private final BrainSTEMRobot robot;
    private MotorCacher turretMotor, shooterLowMotor, shooterHighMotor;
    private ServoImplCacher hoodLeftServo, hoodRightServo;
    public final Vector2d corner;
    public Vector3d goalPosIn;
    public double impactAngleRad;
    public Pose2d futureRobotPose;

    public double absoluteTargetAngleRad;

    public double currentLookAhead;
    public double[] prevLookAheads;

    public Vector2d robotVelAtExitPosIps;
    public double robotSpeedAtExitPosIps;
    public Vector3d actualTargetExitVelMps;
    public double actualTargetExitSpeedMps;
    public double ballTargetExitSpeedMps;
    public double efficiencyCoef;

    public double filteredShooterSpeedTps, rawShooterSpeedTps;
    public double curExitSpeedMps;
    public double ballExitAngleRad, hoodExitAngleRad;
    public double[] physicsExitAngleRads;
    public Vector2d ballExitPos;
    public double exitPosGoalDistIn;
    public Pose2d turretPose;

    private double curTimeMs;
    public double dt;
    public OdoInfo odoVel;
    public ShooterLookup lookupTable;
    public double relGoalHeightM;
    public ShootingSystem(HardwareMap hardwareMap, BrainSTEMRobot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;

        if(robot != null)
            ballExitPos = ShootingMath.getExitPositionInches(ShootingMath.getTurretPose(robot.drive.localizer.getPose(), 0), ballExitAngleRad);

        initTurret();
        initShooter();
        initHood();
        initMisc();
        curTimeMs = System.currentTimeMillis();
        lookupTable = new ShooterLookup();
        efficiencyCoef = 0.39;


        distState = Dist.NEAR;
        if(BrainSTEMRobot.alliance == Alliance.BLUE) {
            corner = new Vector2d(-72, -72);
            nearGoalPos = new Vector3d(goalParams.nearBlueX, goalParams.nearHeight, goalParams.nearBlueY);
            midGoalPos = new Vector3d(goalParams.midBlueX, goalParams.midHeight, goalParams.midBlueY);
            farGoalPos = new Vector3d(goalParams.farBlueX, goalParams.farHeight, goalParams.farBlueY);
        }
        else {
            corner = new Vector2d(-72, 72);
            nearGoalPos = new Vector3d(goalParams.nearRedX, goalParams.nearHeight, goalParams.nearRedY);
            midGoalPos = new Vector3d(goalParams.midRedX, goalParams.midHeight, goalParams.midRedY);
            farGoalPos = new Vector3d(goalParams.farRedX, goalParams.farHeight, goalParams.farRedY);
        }

        physicsExitAngleRads = new double[generalParams.numApproximations];
    }
    private void initTurret() {
        DcMotorEx rawTurretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        rawTurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rawTurretMotor.setPower(0);
        this.turretMotor = new MotorCacher(rawTurretMotor);
    }
    private void initShooter() {
        DcMotorEx rawShooterMotorLow = hardwareMap.get(DcMotorEx.class, "lowShoot");
        rawShooterMotorLow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rawShooterMotorLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rawShooterMotorLow.setDirection(DcMotorSimple.Direction.FORWARD);
        rawShooterMotorLow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.shooterLowMotor = new MotorCacher(rawShooterMotorLow);

        DcMotorEx rawShooterMotorHigh = hardwareMap.get(DcMotorEx.class, "highShoot");
        rawShooterMotorHigh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rawShooterMotorHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rawShooterMotorHigh.setDirection(DcMotorSimple.Direction.REVERSE);
        rawShooterMotorHigh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.shooterHighMotor = new MotorCacher(rawShooterMotorHigh);
    }
    private void initHood() {
        ServoImplEx rawHoodLeftServo = hardwareMap.get(ServoImplEx.class, "hoodLeft");
        rawHoodLeftServo.setPwmRange(new PwmControl.PwmRange(hoodParams.downPWM, hoodParams.upPWM));
        this.hoodLeftServo = new ServoImplCacher(rawHoodLeftServo);

        ServoImplEx rawHoodRightServo = hardwareMap.get(ServoImplEx.class, "hoodRight");
        rawHoodRightServo.setPwmRange(new PwmControl.PwmRange(hoodParams.downPWM, hoodParams.upPWM));
        this.hoodRightServo = new ServoImplCacher(rawHoodRightServo);
    }
    private void initMisc() {
        prevLookAheads = new double[generalParams.lookAheadAvgNum];
        for(int i = 0; i < generalParams.lookAheadAvgNum; i++)
            prevLookAheads[i] = 0;
    }
    public void updateInfo(boolean useTurretLookAhead) {
        updateLookAheadTime(useTurretLookAhead);

        double prevTimeMs = curTimeMs;
        curTimeMs = System.currentTimeMillis();
        dt = (curTimeMs - prevTimeMs) / 1000;

        hoodLeftServo.updateProperties();
        hoodRightServo.updateProperties();

        Pose2d robotPose = robot.drive.localizer.getPose();
        futureRobotPose = robot.drive.pinpoint().getNextPoseSimple(currentLookAhead);

        updateGoalProperties(robotPose.position);

        updateTurretProperties(robotPose);
        shooterHighMotor.updateInfo();
        shooterLowMotor.updateInfo();

        rawShooterSpeedTps = (shooterHighMotor.getVelTps() + shooterLowMotor.getVelTps()) * 0.5;
        double a = generalParams.shooterTau == 0 ? 0 : Math.exp(-dt / generalParams.shooterTau);
        filteredShooterSpeedTps = filteredShooterSpeedTps * a + rawShooterSpeedTps * (1-a);

        Pose2d futureTurretPose = ShootingMath.getTurretPose(futureRobotPose, 0);
        Vector2d futureExitPos = ShootingMath.getExitPositionInches(futureTurretPose, ballExitAngleRad);
        double desiredBallDir = Math.atan2(goalPosIn.z - futureExitPos.y, goalPosIn.x - futureExitPos.x);

        Vector2d robotVel = robotVelAtExitPosIps.times(testingParams.enableShootingWhileMoving ? 0.0254 : 0);
        if(testingParams.usingLookup)
            updateLookupProperties(desiredBallDir, robotVel);
        else
            updatePhysicsProperties(desiredBallDir, robotVel);
    }

    // pro: yes velocity-based hood adjustment
    // con: math is weird
    private void updatePhysicsProperties(double desiredBallDir, Vector2d robotVel) {
        // get delta y of projectory (need approximate exit height of the ball)
        double exitHeightM = ShootingMath.approximateExitHeightM(distState == Dist.NEAR);
        relGoalHeightM = (goalPosIn.y * 0.0254 - exitHeightM);
        double exitPosGoalDistM = exitPosGoalDistIn * 0.0254;

        double[] launchVector = ShootingMath.calculateLaunchVector(exitPosGoalDistM, relGoalHeightM, impactAngleRad);
        ballTargetExitSpeedMps = launchVector[0];
        efficiencyCoef = calcEfficiencyCoef(launchVector[1]); // initial guess for efficiency coefficient
        ballExitAngleRad = launchVector[1];
        curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);

        if(testingParams.dynamicHood) {
            // determining whether to use high arc or low arc
            double highArcExitAng = ShootingMath.calculateBallExitAngleRad(true, relGoalHeightM, exitPosGoalDistM, curExitSpeedMps);
            if(highArcExitAng != -1) {
                double lowArcExitAng = ShootingMath.calculateBallExitAngleRad(false, relGoalHeightM, exitPosGoalDistM, curExitSpeedMps);
                double highArcImpactAng = ShootingMath.calculateImpactAngle(exitPosGoalDistM, relGoalHeightM, curExitSpeedMps, highArcExitAng);
                double lowArcImpactAng = ShootingMath.calculateImpactAngle(exitPosGoalDistM, relGoalHeightM, curExitSpeedMps, lowArcExitAng);
                usingHighArc = Math.abs(highArcImpactAng - impactAngleRad) < Math.abs(lowArcImpactAng - impactAngleRad);

                // estimating hood ang and current shooter speed
                physicsExitAngleRads[0] = usingHighArc ? highArcExitAng : lowArcExitAng;
                ballExitAngleRad = physicsExitAngleRads[0];
                efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
                curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);

                for (int i = 1; i < generalParams.numApproximations; i++) {
                    physicsExitAngleRads[i] = ShootingMath.calculateBallExitAngleRad(usingHighArc, relGoalHeightM, exitPosGoalDistM, curExitSpeedMps);
                    if(physicsExitAngleRads[i] == -1)
                        break;
                    ballExitAngleRad = physicsExitAngleRads[i];
                    efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
                    curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
                }
            }
            else
                Arrays.fill(physicsExitAngleRads, -1);
        }

        // old code with no flexibility for shooting while moving
        actualTargetExitSpeedMps = ballTargetExitSpeedMps;
        hoodExitAngleRad = ballExitAngleRad;
        absoluteTargetAngleRad = desiredBallDir;

        // allows for shooting while moving
        // TODO: FIX TS IS DOESN'T WORK
//        double speed = ballTargetExitSpeedMps;
//        actualTargetExitVelMps = ShootingMath.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, speed, robotVel);
    }

    // pro: easy to tune
    // con: no velocity-based hood adjustment
    private void updateLookupProperties(double desiredBallDir, Vector2d robotVel) {
        // getting lookup properties
        double lookupDist = Range.clip(exitPosGoalDistIn, lookupDistsI[0] + 0.01, lookupDistsI[lookupDistsI.length-1] - 0.01);
        ballExitAngleRad = lookupTable.lookupExitAngleRad(lookupDist);
        ballTargetExitSpeedMps = lookupTable.lookupVelocityMetersPerSec(lookupDist);

        // allows for shooting while moving
        actualTargetExitVelMps = ShootingMath.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, ballTargetExitSpeedMps, robotVel);

        double baseLength = Math.hypot(actualTargetExitVelMps.x, actualTargetExitVelMps.z);
        hoodExitAngleRad = Math.atan2(actualTargetExitVelMps.y, baseLength);
        hoodExitAngleRad = Range.clip(hoodExitAngleRad, hoodParams.minExitAngRad, hoodParams.maxExitAngRad);
        absoluteTargetAngleRad = Math.atan2(actualTargetExitVelMps.z, actualTargetExitVelMps.x);

        efficiencyCoef = calcEfficiencyCoef(hoodExitAngleRad);
        curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
        actualTargetExitSpeedMps = Math.hypot( baseLength, actualTargetExitVelMps.y );
    }
    public void sendHardwareInfo() {
        turretMotor.sendInfo();
        shooterHighMotor.sendInfo();
        shooterLowMotor.sendInfo();
        if(Math.abs(hoodLeftServo.getPosition() - hoodLeftServo.getTargetPosition()) > hoodParams.resolution) {
            hoodLeftServo.sendInfo();
            hoodRightServo.sendInfo();
        }
    }
    private void updateTurretProperties(Pose2d robotPose) {
        turretMotor.updateInfo();
        turretPose = ShootingMath.getTurretPose(robotPose, robot.turret.currentRelativeAngleRad);
        ballExitPos = ShootingMath.getExitPositionInches(turretPose, ballExitAngleRad);
        odoVel = !robot.drive.pinpoint().previousVelocities.isEmpty() ? robot.drive.pinpoint().previousVelocities.get(0) : new OdoInfo(0, 0, 0);
        Vector2d robotVelCm = new Vector2d(odoVel.x, odoVel.y);
        Vector2d relativeExitPos = ballExitPos.minus(robotPose.position);
        Vector2d robotTanVel = new Vector2d(-relativeExitPos.y, relativeExitPos.x*1).times(odoVel.headingRad); // v = r * w
        robotVelAtExitPosIps = robotVelCm.plus(robotTanVel);
        robotSpeedAtExitPosIps = Math.hypot(robotVelAtExitPosIps.x, robotVelAtExitPosIps.y);
        robotSpeedAtExitPosIps = Math.hypot(robotVelAtExitPosIps.x, robotVelAtExitPosIps.y);

        double deltaX = goalPosIn.x - ballExitPos.x;
        double deltaY = goalPosIn.z - ballExitPos.y;
        exitPosGoalDistIn = Math.hypot(deltaX, deltaY);
    }

    private void updateLookAheadTime(boolean useLookAhead) {
        double rawLookAhead = useLookAhead ? generalParams.rawLookAheadTime : 0;
        for(int i = prevLookAheads.length-1; i > 0; i--)
            prevLookAheads[i] = prevLookAheads[i-1];
        prevLookAheads[0] = rawLookAhead;
        currentLookAhead = getAvgLookAheads();
    }
    private double getAvgLookAheads() {
        double sum = 0;
        for (double prevLookAhead : prevLookAheads) sum += prevLookAhead;
        return sum / prevLookAheads.length;
    }
    private void updateGoalProperties(Vector2d robotPos) {
        if(robotPos.x > 24) {
            distState = Dist.FAR;
            goalPosIn = farGoalPos;
            impactAngleRad = goalParams.farImpactAng;
        }
        double distToCorner = Math.hypot(corner.x - robotPos.x, corner.y - robotPos.y);
        if(distToCorner > goalParams.nearStateThreshold) {
            distState = Dist.MID;
            goalPosIn = midGoalPos;
            impactAngleRad = goalParams.midImpactAng;
        }
        else {
            distState = Dist.NEAR;
            goalPosIn = nearGoalPos;
            impactAngleRad = goalParams.nearImpactAng;
        }
    }

    public void printInfo(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addLine("SHOOTING SYSTEM-------");
        telemetry.addData("efficiency coef", efficiencyCoef);
        telemetry.addData("absolute turret target rad", absoluteTargetAngleRad);
        telemetry.addData("robot-relative target exit speed mps", actualTargetExitSpeedMps);
        telemetry.addData("ball exit angle rad", ballExitAngleRad);
        telemetry.addData("physics exit angle rad", MathUtils.format3(physicsExitAngleRads));
        telemetry.addData("ball meters from goal", exitPosGoalDistIn * 0.0254);
        telemetry.addData("ball inches from goal", exitPosGoalDistIn);
        telemetry.addData("absolute target exit speed mps", ballTargetExitSpeedMps);
        telemetry.addData("dt", dt);
        telemetry.addLine();
        telemetry.addData("height to target meters", relGoalHeightM);
        telemetry.addData("dist state", distState);
    }

    public int getTurretEncoder() {
        return turretMotor.getCurrentPosition();
    }
    public int getTurretEncoderRaw() {
        return turretMotor.getCurrentPositionRaw();
    }
    public void resetTurretEncoder() {
        turretMotor.resetEncoders();
    }
    public void setTurretPower(double p) {
        turretMotor.setPower(p);
    }
    public double getTurretPower() {
        return turretMotor.getPower();
    }
    public double getTurretVelTps() {
        return turretMotor.getVelTps();
    }
    public void setShooterPower(double p) {
        shooterHighMotor.setPower(p);
        shooterLowMotor.setPower(p);
    }
    public double calcEfficiencyCoef(double ballExitAngleRad) {
        double rawE = generalParams.efficiencyCoefM * ballExitAngleRad + generalParams.efficiencyCoefB;
        return Range.clip(generalParams.minEfficiencyCoef, rawE, generalParams.maxEfficiencyCoef);
    }
    public void setShooterPowerRaw(double p) {
        shooterHighMotor.setPowerRaw(p);
        shooterLowMotor.setPowerRaw(p);
    }
    public double getShooterPower() {
        return shooterHighMotor.getPower();
    }
    public double getShooterHighVelTps() {
        return shooterHighMotor.getVelTps();
    }
    public double getShooterLowVelTps() {
        return shooterLowMotor.getVelTps();
    }
    public double getPrevShooterVelTps() {
        return (shooterHighMotor.getPrevVelTps() + shooterLowMotor.getPrevVelTps()) * 0.5;
    }

    public void setHoodPosition(double p) {
        hoodLeftServo.setPosition(p);
        hoodRightServo.setPosition(p);
    }
    public double getHoodPosition() {
        return hoodLeftServo.getPosition();
    }
}
