package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterLookup.lookupDistsI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.math.Vector3d;

import java.util.Arrays;

@Config
public class ShootingSystem {
    public static class TestingParams {
        public boolean usingLookup = false;
        public boolean enableShootingWhileMovingNear = true, enableShootingWhileMovingFar = false;
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
        public double nearImpactAng = Math.toRadians(-30), midImpactAng = -Math.toRadians(25), farImpactAng = Math.toRadians(-25);
        public double nearStateThreshold = 58;
        public double classifierX = -24, classifierY = 68;
    }
    public static class HoodParams {
        public double minExitAngRad = Math.toRadians(35), maxExitAngRad = Math.toRadians(85);
    }
    public static class GeneralParams {
        public double approxNearExitAngRad = Math.toRadians(53), approxFarExitAngRad = Math.toRadians(37);
        public int numApproximations = 4;
        // efficiency coef regression: y=-0.0766393x+0.446492
        public double efficiencyCoefM = -0.0766393, efficiencyCoefB = 0.446492;
        public double minEfficiencyCoef = 0.3327, maxEfficiencyCoef = 0.4000;
        public double maxShootWhileMovingSpeed = 0.6;

        // estimated accel thresholds: position: 20, heading: 5
    }
    public static TestingParams testingParams = new TestingParams();
    public static GoalParams goalParams = new GoalParams();
    public static HoodParams hoodParams = new HoodParams();
    public static GeneralParams generalParams = new GeneralParams();

    public enum DistState {
        NEAR, MID, FAR
    }
    private DistState distState;
    private final Vector3d nearGoalPos, midGoalPos, farGoalPos;
    private final Vector2d corner;
    private Vector3d goalPosIn;
    private double relGoalHeightM;
    private double impactAngleRad;


    private Vector2d robotVelAtExitPosIps;

    private Vector3d targetShooterVelVelMps;
    private double targetShooterSpeedMps, curShooterSpeedMps;
    private double ballAbsTargetExitSpeedMps;
    private double turretAbsoluteTargetAngleRad, turretRelTargetAngleRad;
    private double turretTargetAngularVelocity;
    private double efficiencyCoef, idealEfficiencyCoef;

    private double targetShooterSpeedTps;
    private double ballExitAngleRad, hoodExitAngleRad;
    private final double[] physicsExitAngleRads;
    private Vector2d ballExitPos, futureBallExitPos;
    private Vector2d exitPosRelativeToGoal;
    private double exitPosGoalDistIn, futureExitPosGoalDistIn;
    private double classifierTurretTargetAngle;
    private Pose2d absoluteTurretPose, futureTurretPose;

    private final ShooterLookup lookupTable;

    private final Turret turret;
    private final Shooter shooter;
    private double nearVelocityAdjustment, farVelocityAdjustment;
    private double nearEncoderAdjustment, farEncoderAdjustment;

    public enum TurretState {
        CENTER, TRACKING, CLASSIFIER
    }

    public enum ShooterState {
        OFF, UPDATE
    }
    private TurretState turretState;
    private ShooterState shooterState;
    public ShootingSystem(HardwareMap hardwareMap, Telemetry telemetry, Pose2d robotPose) {
        turret = new Turret(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);

        turretState = TurretState.CENTER;
        shooterState = ShooterState.OFF;

        absoluteTurretPose = ShootingMath.getTurretPose(robotPose, turret.getAbsAngleRad(robotPose.heading.toDouble()));
        ballExitPos = ShootingMath.getExitPositionInches(absoluteTurretPose, ballExitAngleRad);

        lookupTable = new ShooterLookup();
        efficiencyCoef = 0.39;


        distState = DistState.NEAR;
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
    public void updateProperties(double dt, Pose2d robotPose, Pose2d futureRobotPose, OdoInfo odoVel, boolean shootingWhileMoving) {
        turret.updateProperties();
        shooter.updateProperties(dt);
        absoluteTurretPose = ShootingMath.getTurretPose(robotPose, turret.getAbsAngleRad(robotPose.heading.toDouble()));
        futureTurretPose = ShootingMath.getTurretPose(futureRobotPose, turret.getRelAngleRad());
        updateGoalProperties(robotPose.position);

        updateTurretProperties(robotPose, absoluteTurretPose, futureTurretPose, odoVel);

        double desiredBallDir = Math.atan2(goalPosIn.z - futureBallExitPos.y, goalPosIn.x - futureBallExitPos.x);

        Vector2d robotExitPosVel = robotVelAtExitPosIps.times(shootingWhileMoving ? 0.0254 : 0);
        if(testingParams.usingLookup)
            updateLookupProperties(desiredBallDir, robotExitPosVel, shooter.getFilteredShooterSpeed());
        else
            updatePhysicsProperties(desiredBallDir, shootingWhileMoving, robotExitPosVel, shooter.getFilteredShooterSpeed());
        turretRelTargetAngleRad = turretAbsoluteTargetAngleRad - robotPose.heading.toDouble();
        targetShooterSpeedTps = ShootingMath.exitMpsToMotorTicksPerSec(targetShooterSpeedMps, idealEfficiencyCoef);

        double robotSpeedPerpToGoal = new Vector2d(odoVel.x, odoVel.y).dot(new Vector2d(-exitPosRelativeToGoal.y, exitPosRelativeToGoal.x*1).div(exitPosGoalDistIn));
        turretTargetAngularVelocity = -odoVel.headingRad + robotSpeedPerpToGoal / exitPosGoalDistIn; // absolute w = w1 + v/r
    }
    public void updateState(double dt, boolean enableShooter, boolean enableTurret) {
        if(enableTurret) {
            switch (turretState) {
                case CENTER:
                    turret.setTarget(dt, 0);
                    break;
                case TRACKING:
                    double encoderAdjustment = distState == DistState.NEAR ? nearEncoderAdjustment : farEncoderAdjustment;
                    turret.setTarget(turretRelTargetAngleRad + turret.ticksToAngle(encoderAdjustment), turretTargetAngularVelocity);
                case CLASSIFIER:
                    turret.setTarget(dt, classifierTurretTargetAngle);
            }
            turret.setPower(turret.calculateTurretPower());
        }
        else
            turret.setPower(0);

        if(enableShooter) {
            switch (shooterState) {
                case OFF:
                    shooter.setShooterPower(0);
                    break;
                case UPDATE:
                    double adjustment = distState == DistState.FAR ? farVelocityAdjustment : nearVelocityAdjustment;
                    shooter.updateTarget(targetShooterSpeedTps + adjustment, hoodExitAngleRad);
                    break;
            }
        }
        else
            shooter.setShooterPower(0);
    }

    // pro: yes velocity-based hood adjustment
    // con: math is weird
    private void updatePhysicsProperties(double desiredBallDir, boolean shootWhileMoving, Vector2d robotExitPosVel, double filteredShooterSpeedTps) {
        // get delta y of projectory (need approximate exit height of the ball)
        double exitHeightM = ShootingMath.approximateExitHeightM(distState == DistState.NEAR);
        relGoalHeightM = (goalPosIn.y * 0.0254 - exitHeightM);
        double futureDist = futureExitPosGoalDistIn * 0.0254;

        double[] launchVector = ShootingMath.calculateLaunchVector(futureDist, relGoalHeightM, impactAngleRad);

        ballAbsTargetExitSpeedMps = launchVector[0];
        idealEfficiencyCoef = calcEfficiencyCoef(launchVector[1]);

        if(shootWhileMoving) {
            ballExitAngleRad = launchVector[1];

            targetShooterVelVelMps = ShootingMath.calculateActualTargetExitVel(desiredBallDir, launchVector[1], launchVector[0], robotExitPosVel);
            double baseLength = Math.hypot(targetShooterVelVelMps.x, targetShooterVelVelMps.z);
            hoodExitAngleRad = Range.clip(Math.atan2(targetShooterVelVelMps.y, baseLength), hoodParams.minExitAngRad, hoodParams.maxExitAngRad);
            turretAbsoluteTargetAngleRad = Math.atan2(targetShooterVelVelMps.z, targetShooterVelVelMps.x);

            efficiencyCoef = calcEfficiencyCoef(hoodExitAngleRad);
            targetShooterSpeedMps = Math.hypot( baseLength, targetShooterVelVelMps.y );
        }
        else {
            efficiencyCoef = idealEfficiencyCoef; // initial guess for efficiency coefficient
            ballExitAngleRad = launchVector[1];
            curShooterSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);

            if (testingParams.dynamicHood) {
                // determining whether to use high arc or low arc
                double highArcExitAng = ShootingMath.calculateBallExitAngleRad(true, relGoalHeightM, futureDist, curShooterSpeedMps);
                if (highArcExitAng != -1) {
                    double lowArcExitAng = ShootingMath.calculateBallExitAngleRad(false, relGoalHeightM, futureDist, curShooterSpeedMps);
                    double highArcImpactAng = ShootingMath.calculateImpactAngle(futureDist, relGoalHeightM, curShooterSpeedMps, highArcExitAng);
                    double lowArcImpactAng = ShootingMath.calculateImpactAngle(futureDist, relGoalHeightM, curShooterSpeedMps, lowArcExitAng);
                    boolean usingHighArc = Math.abs(highArcImpactAng - impactAngleRad) < Math.abs(lowArcImpactAng - impactAngleRad);

                    // estimating hood ang and current shooter speed
                    physicsExitAngleRads[0] = usingHighArc ? highArcExitAng : lowArcExitAng;
                    ballExitAngleRad = physicsExitAngleRads[0];
                    efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
                    curShooterSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);

                    for (int i = 1; i < generalParams.numApproximations; i++) {
                        physicsExitAngleRads[i] = ShootingMath.calculateBallExitAngleRad(usingHighArc, relGoalHeightM, futureDist, curShooterSpeedMps);
                        if (physicsExitAngleRads[i] == -1) {
                            for (int j = i + 1; j < generalParams.numApproximations; j++)
                                physicsExitAngleRads[j] = -1;
                            break;
                        }
                        ballExitAngleRad = physicsExitAngleRads[i];
                        efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
                        curShooterSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
                    }
                } else
                    Arrays.fill(physicsExitAngleRads, -1);
            }

            targetShooterSpeedMps = ballAbsTargetExitSpeedMps;
            if(physicsExitAngleRads[0] != -1)
                hoodExitAngleRad = ballExitAngleRad;
            turretAbsoluteTargetAngleRad = desiredBallDir;
        }
    }

    // pro: easy to tune
    // con: no velocity-based hood adjustment
    private void updateLookupProperties(double desiredBallDir, Vector2d robotVel, double filteredShooterSpeedTps) {
        // getting lookup properties
        double lookupDist = Range.clip(exitPosGoalDistIn, lookupDistsI[0] + 0.01, lookupDistsI[lookupDistsI.length-1] - 0.01);
        ballExitAngleRad = lookupTable.lookupExitAngleRad(lookupDist);
        ballAbsTargetExitSpeedMps = lookupTable.lookupVelocityMetersPerSec(lookupDist);

        // allows for shooting while moving
        targetShooterVelVelMps = ShootingMath.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, ballAbsTargetExitSpeedMps, robotVel);
        double baseLength = Math.hypot(targetShooterVelVelMps.x, targetShooterVelVelMps.z);
        hoodExitAngleRad = Range.clip(Math.atan2(targetShooterVelVelMps.y, baseLength), hoodParams.minExitAngRad, hoodParams.maxExitAngRad);
        turretAbsoluteTargetAngleRad = Math.atan2(targetShooterVelVelMps.z, targetShooterVelVelMps.x);

        efficiencyCoef = calcEfficiencyCoef(hoodExitAngleRad);
        idealEfficiencyCoef = efficiencyCoef;
        targetShooterSpeedMps = Math.hypot( baseLength, targetShooterVelVelMps.y );
    }
    private void updateTurretProperties(Pose2d robotPose, Pose2d absoluteTurretPose, Pose2d futureTurretPose, OdoInfo odoVel) {
        double approxBallExitAng = distState == DistState.FAR ? generalParams.approxFarExitAngRad : generalParams.approxNearExitAngRad;
        ballExitPos = ShootingMath.getExitPositionInches(absoluteTurretPose, approxBallExitAng);
        futureBallExitPos = ShootingMath.getExitPositionInches(futureTurretPose, approxBallExitAng);

        Vector2d robotVelCm = new Vector2d(odoVel.x, odoVel.y);
        Vector2d relativeExitPos = ballExitPos.minus(robotPose.position);
        Vector2d robotTanVel = new Vector2d(-relativeExitPos.y, relativeExitPos.x*1).times(odoVel.headingRad); // v = r * w
        robotVelAtExitPosIps = robotVelCm.plus(robotTanVel);

        double deltaX = goalPosIn.x - ballExitPos.x;
        double deltaY = goalPosIn.z - ballExitPos.y;
        exitPosRelativeToGoal = new Vector2d(deltaX, deltaY);
        exitPosGoalDistIn = Math.hypot(deltaX, deltaY);

        double futureDx = goalPosIn.x - futureBallExitPos.x;
        double futureDy = goalPosIn.z - futureBallExitPos.y;
        futureExitPosGoalDistIn = Math.hypot(futureDx, futureDy);

        double classifierDx = goalParams.classifierX - robotPose.position.x;
        double classifierDy = goalParams.classifierY - robotPose.position.y;
        double classifierAngle = Math.atan2(classifierDy, classifierDx);
        classifierTurretTargetAngle = classifierAngle - robotPose.heading.toDouble();
    }
    private void updateGoalProperties(Vector2d robotPos) {
        double distToCorner = Math.hypot(corner.x - robotPos.x, corner.y - robotPos.y);
        if(robotPos.x > 24) {
            distState = DistState.FAR;
            goalPosIn = farGoalPos;
            impactAngleRad = goalParams.farImpactAng;
        }
        else if(distToCorner > goalParams.nearStateThreshold) {
            distState = DistState.MID;
            goalPosIn = midGoalPos;
            impactAngleRad = goalParams.midImpactAng;
        }
        else {
            distState = DistState.NEAR;
            goalPosIn = nearGoalPos;
            impactAngleRad = goalParams.nearImpactAng;
        }
    }

    public void printInfo(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addLine("SHOOTING SYSTEM-------");

        telemetry.addData("efficiency coef", efficiencyCoef);
        telemetry.addData("absolute turret target rad", turretAbsoluteTargetAngleRad);
        telemetry.addData("robot-relative target exit speed mps", targetShooterSpeedMps);
        telemetry.addData("ball exit angle rad", ballExitAngleRad);
        telemetry.addData("physics exit angle rad", MathUtils.format3(physicsExitAngleRads));
        telemetry.addData("ball meters from goal", exitPosGoalDistIn * 0.0254);
        telemetry.addData("ball inches from goal", exitPosGoalDistIn);
        telemetry.addData("future ball meters from goal", futureExitPosGoalDistIn * 0.0254);
        telemetry.addData("future ball inches from goal", futureExitPosGoalDistIn);
        telemetry.addData("absolute target exit speed mps", ballAbsTargetExitSpeedMps);
        telemetry.addLine();
        telemetry.addData("rel height to target meters", relGoalHeightM);
        telemetry.addData("dist state", distState);
    }
    public double calcEfficiencyCoef(double ballExitAngleRad) {
        double rawE = generalParams.efficiencyCoefM * ballExitAngleRad + generalParams.efficiencyCoefB;
        return Range.clip(generalParams.minEfficiencyCoef, rawE, generalParams.maxEfficiencyCoef);
    }
    public void changeTurretEncoderAdjustment(double adjustment) {
        if(distState == DistState.FAR)
            farEncoderAdjustment += adjustment;
        else
            nearEncoderAdjustment += adjustment;
    }
    public void changeShooterTicksAdjustment(double adjustment) {
        if(distState == DistState.FAR)
            farVelocityAdjustment += adjustment;
        else
            nearVelocityAdjustment += adjustment;
    }

    public TurretState getTurretState() {
        return turretState;
    }
    public void setTurretState(TurretState turretState) {
        this.turretState = turretState;
    }
    public ShooterState getShooterState() {
        return shooterState;
    }
    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }
    public DistState getDistState() {
        return distState;
    }
    public Turret getTurret() {
        return turret;
    }
    public Shooter getShooter() {
        return shooter;
    }
    public double getTargetShooterSpeedTps() {
        return targetShooterSpeedTps;
    }
    public Pose2d getAbsoluteTurretPose() {
        return absoluteTurretPose;
    }
    public Vector2d getBallExitPos() {
        return ballExitPos;
    }
    public Vector2d get2dGoalPos() {
        return new Vector2d(goalPosIn.x, goalPosIn.z);
    }
    public double getTurretAbsoluteTargetAngle() {
        return turretAbsoluteTargetAngleRad;
    }
    public boolean physicsShotPossible() {
        return physicsExitAngleRads[0] != -1;
    }
    public double getShooterErrorMps() {
        return targetShooterSpeedMps - curShooterSpeedMps;
    }
}
