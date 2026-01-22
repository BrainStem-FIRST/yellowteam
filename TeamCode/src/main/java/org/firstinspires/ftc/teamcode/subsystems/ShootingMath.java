package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Turret.turretParams;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.math.Vec;

@Config
public class ShootingMath {
    // stores all parameters of the shooter/hood/turret system
    public static class ShooterSystemParams {
        public double flywheelHeightMeters = 0.2413;
        public double highArcTargetHeightInches = 38, lowArcNearTargetHeightInches = 46.5, lowArcFarTargetHeightInches = 46.5;
        public double flywheelOffsetFromTurretInches = 2.4783465;
        public double flywheelRadiusMeters = 0.0445;
        public double ballRadiusMeters = 0.064;
        public double powerEfficiencyCoefficient = 0.45; // actual exit velocity / theoretical exit velocity
        public double shooterMotorTicksPerRev = 28;
        // 28 motor ticks = one revolution
        // 38.5 flywheel ticks = one revolution
        public double flywheelTicksPerRev = 38.5;

        // slope and y intercept of distance-exit speed regression - ultimate value takes the max value between these 2
        public double highShotTicksPerSecSlope = 3.8, highShotTicksPerSecYInt = 1100;
        public double lowShotNearZoneTicksPerSec = 5.5, lowShotNearZoneTicksPerSecYInt = 1010;
        public double lowShotFarZoneTicksPerSecSlope = 6.46622, lowShotFarZoneTicksPerSecYInt = 829.67635; //y=6.46622x+829.67635
        public double flywheelSpeedRelativeVelocityMultiplier = 1;
        public double closeToFarZoneThresholdInches = 130;
        public double highToLowArcThresholdInches = 56; // old: 62
        public double highArcGoalOffsetInches = 0, lowArcNearGoalOffsetInches = 0, lowArcFarGoalOffsetInches = 0;
    }
    public static class HoodSystemParams {
        public double restingDistanceMm = 82;
        public double hoodPivotAngleOffsetFromHoodExitAngleDeg = 7.8;
        public double servoRangeMm = 30;
        public double minAngleDeg = 15, maxAngleDeg = 55;
        public double hoodAngleRelativeVelocityMultiplier = 1;
        public double highArcHoodAngleDegEstimation = 20, lowArcHoodAngleDegEstimation = 45;
    }
    public static class TurretSystemParams {
        // the ball exit position must be traveling at a speed (in/sec) greater than this to account for its relative velocity
        public double predictVelocityExitSpeedThresholdMps = 0.05;
        public double predictVelocityMultiplier = 1;
        public double maxAngleDeg = 90, minAngleDeg = -90;
    }
    public static ShooterSystemParams shooterSystemParams = new ShooterSystemParams();
    public static HoodSystemParams hoodSystemParams = new HoodSystemParams();
    public static TurretSystemParams turretSystemParams = new TurretSystemParams();
    public static boolean enableRelativeVelocity = true;

    // what this is used for as of 11/23
    // calculating distance from exit position to goal in updateShooterSystem
    // calculating current and future exit position pose for turret relative velocity correction
    // dashboard field telemetry and raw subsystem testing
    public static Vector2d calculateExitPositionInches(Pose2d turretPose, double ballExitAngleRad) {
        double hoodAngleRad = Math.PI * 0.5 - ballExitAngleRad;
        double shooterCombinedRadiusInches = (shooterSystemParams.flywheelRadiusMeters + shooterSystemParams.ballRadiusMeters) / 0.0254;
        double offsetFromTurretInches = shooterSystemParams.flywheelOffsetFromTurretInches - Math.cos(hoodAngleRad) * shooterCombinedRadiusInches;

        return new Vector2d(
                turretPose.position.x + offsetFromTurretInches * Math.cos(turretPose.heading.toDouble()),
                turretPose.position.y + offsetFromTurretInches * Math.sin(turretPose.heading.toDouble())
        );
    }

    // approximates ball exit height to avoid hood jitter in tele
    public static double approximateExitHeightMeters(boolean useHighArc) {
        double hoodAngleDeg = useHighArc ? hoodSystemParams.highArcHoodAngleDegEstimation : hoodSystemParams.lowArcHoodAngleDegEstimation;
        double exitAngleRad = Math.toRadians(90 - hoodAngleDeg);
        return calculateExactExitHeightMeters(exitAngleRad);
    }

    // calculates exact ball exit height
    public static double calculateExactExitHeightMeters(double exitAngleRad) {
        double hoodAngleRad = Math.PI * 0.5 - exitAngleRad;
        return shooterSystemParams.flywheelHeightMeters + (shooterSystemParams.flywheelRadiusMeters + shooterSystemParams.ballRadiusMeters) * Math.sin(hoodAngleRad);
    }
    // finds exit speed of ball (meters per sec) if flywheel is spinning at ticksPerSec
    public static double ticksPerSecToExitSpeedMps(double motorTicksPerSec, double efficiencyCoefficient) {
        double motorRevPerSec = motorTicksPerSec / shooterSystemParams.shooterMotorTicksPerRev;
        double motorAngularVel = motorRevPerSec * 2 * Math.PI;
        double flywheelAngularVel = motorAngularVel * 16 / 18;
        double flywheelTangentialVel = flywheelAngularVel * shooterSystemParams.flywheelRadiusMeters;
        return flywheelTangentialVel * efficiencyCoefficient;
    }

    // finds required speed of flywheel (encoder ticks per sec) to shoot the ball at a speed of mps
    public static double exitMpsToMotorTicksPerSec(double ballExitMps, double efficiencyCoefficient) {
        double flywheelTangentialVel = ballExitMps / efficiencyCoefficient;
        double flywheelAngularVel = flywheelTangentialVel / shooterSystemParams.flywheelRadiusMeters;
        double motorAngularVel = flywheelAngularVel * 18 / 16;
        double motorRevPerSec = motorAngularVel / (2 * Math.PI);
        return motorRevPerSec * shooterSystemParams.shooterMotorTicksPerRev;
    }

    // calculates the component of robot velocity that is towards the goal
    // TODO - USE BALL EXIT VELOCITY INSTEAD OF ROBOT VELOCITY
//    public static double calculateSpeedTowardGoalMps(Pose2d targetPose, Vector2d ballExitPosition, OdoInfo mostRecentVelocity) {
//        double dx = targetPose.position.x - ballExitPosition.x;
//        double dy = targetPose.position.y - ballExitPosition.y;
//
//        double distanceFromGoal = Math.hypot(dx, dy);
//        double ux = dx / distanceFromGoal;
//        double uy = dy / distanceFromGoal;
//
//        double vx = mostRecentVelocity.x;
//        double vy = mostRecentVelocity.y;
//
//        return -(vx * ux + vy * uy) * 0.0254;
//    }

    // calculate flywheel speed (encoder ticks per sec) given a bunch of shooterSystemParams
    // can specify whether to enable or disable relative velocity prediction w/ static constant above
    // TODO - USE BALL EXIT VELOCITY INSTEAD OF ROBOT VELOCITY
//    public static double calculateShooterMotorSpeedTicksPerSec(Telemetry telemetry, Pose2d targetPose, boolean inCloseZone, boolean useHighArc, double inchesFromGoal, Vector2d ballExitPosition, OdoInfo mostRecentVelocity) {
//        double ticksPerSecond;
//        if (inCloseZone) {
//            double highTicksPerSecond = shooterSystemParams.highShotTicksPerSecSlope * inchesFromGoal + shooterSystemParams.highShotTicksPerSecYInt;
//            double lowTicksPerSecond = shooterSystemParams.lowShotNearZoneTicksPerSec * inchesFromGoal + shooterSystemParams.lowShotNearZoneTicksPerSecYInt;
//            ticksPerSecond = Math.max(highTicksPerSecond, lowTicksPerSecond);
//        }
//        else
//            ticksPerSecond = shooterSystemParams.lowShotFarZoneTicksPerSecSlope * inchesFromGoal + shooterSystemParams.lowShotFarZoneTicksPerSecYInt;
//
//        if (!enableRelativeVelocity) {
//            return ticksPerSecond;
//        }
//        // SHOULD return positive value
//        double exitPositionSpeedTowardsGoalMps = ShootingMath.calculateSpeedTowardGoalMps(targetPose, ballExitPosition, mostRecentVelocity);
//        double relativeExitSpeedMps = ticksPerSecond - (exitPositionSpeedTowardsGoalMps * shooterSystemParams.flywheelSpeedRelativeVelocityMultiplier);
//        return exitMpsToMotorTicksPerSec(relativeExitSpeedMps);
//    }
//    public static Vector2d relativeBallExitVelocityMps = new Vector2d(0, 0);
    // calculates where the turret should point given a bunch of shooterSystemParams
    // can specify whether to enable or disable relative velocity prediction w/ static constant above
    public static Vector2d calculateActualTargetExitVel(Vector2d targetPos, Vector2d futureTurretPos, double globalExitSpeedMps, Vector2d robotVelAtExitPosInchesPerSec) {
        Vector2d globalExitVelMps = new Vector2d(targetPos.x - futureTurretPos.x, targetPos.y - futureTurretPos.y);
        globalExitVelMps = globalExitVelMps.div(Math.hypot(globalExitVelMps.x, globalExitVelMps.y)); // normalizing vector
        globalExitVelMps = globalExitVelMps.times(globalExitSpeedMps);

        return globalExitVelMps.minus(robotVelAtExitPosInchesPerSec.times(0.0254));
//        Vector2d actualTargetEXitVel;
//        double targetAngleRad;
//        // account for relative velocity
//        if (enableRelativeVelocity) {
//            // velocity of ball relative to robot = velocity of ball relative to ground - velocity of robot relative to ground
//            actualTargetEXitVel = ;
//            // find angle to shoot at relative velocity
//            targetAngleRad = Math.atan2(relativeBallExitVelocityMps.y, relativeBallExitVelocityMps.x);
//        }
//        // find angle to shoot at without any relative velocity calculations
//        else
//            targetAngleRad = Math.atan2(globalExitVelMps.y, globalExitVelMps.x);
//            actualTargetEXitVel =
//
//        return targetAngleRad;
    }

    // calculates desired exit angle (radians) for ball given a bunch of shooterSystemParams
    // can specify whether to enable or disable relative velocity prediction w/ static constant above
    // TODO - USE BALL EXIT VELOCITY INSTEAD OF ROBOT VELOCITY
    public static double calculateBallExitAngleRad(boolean useHighArc, boolean isNear, double distanceInches, double ballExitSpeedMps) {
        // get the EXACT exit height of the ball (depends on hood position)
        double exitHeightMeters = ShootingMath.approximateExitHeightMeters(useHighArc);
        double targetHeightInches;
        if (useHighArc)
            targetHeightInches = shooterSystemParams.highArcTargetHeightInches;
        else
            targetHeightInches = isNear ? shooterSystemParams.lowArcNearTargetHeightInches : shooterSystemParams.lowArcFarTargetHeightInches;

        double heightToTargetMeters = (targetHeightInches * 0.0254 - exitHeightMeters);

        // Physics formula rearranged for angle: tan(θ) = (v² ± √(v⁴ - g(gx² + 2yv²))) / (gx)
        double g = 9.81;
        double x = distanceInches * 0.0254; // convert inches to meters
        double y = heightToTargetMeters;

        double v = ballExitSpeedMps;
//        if (enableRelativeVelocity) {
//            double exitPositionSpeedTowardsGoalMps = ShootingMath.calculateSpeedTowardGoalMps(targetPose, ballExitPosition, mostRecentVelocity);
//            v += exitPositionSpeedTowardsGoalMps * hoodSystemParams.hoodAngleRelativeVelocityMultiplier;
//        }
        double sign = useHighArc ? 1 : -1;

        double discriminant = v*v*v*v - g*(g*x*x + 2*y*v*v);
        if (discriminant <= 0)
            return -1;

        double tanTheta = (v*v + sign * Math.sqrt(discriminant)) / (g * x);
        return Math.atan(tanTheta);
    }

    // calculates the position to put the linear actuators to on the shooter hood
    public static double calculateHoodServoPosition(double ballExitAngleRadians) {
        double hoodAngleFromXAxisRadians = Math.PI * 0.5 - ballExitAngleRadians;
        double hoodExitAngleDeg = Range.clip(Math.toDegrees(hoodAngleFromXAxisRadians), hoodSystemParams.minAngleDeg, hoodSystemParams.maxAngleDeg);
        double hoodPivotAngleDeg = hoodExitAngleDeg + hoodSystemParams.hoodPivotAngleOffsetFromHoodExitAngleDeg;
        double totalLinearDistanceMm = -0.00125315 * Math.pow(hoodPivotAngleDeg, 2) + 0.858968 * hoodPivotAngleDeg + 63.03978;
        double linearDistanceToExtendMm = totalLinearDistanceMm - hoodSystemParams.restingDistanceMm;
        return linearDistanceToExtendMm / hoodSystemParams.servoRangeMm;
    }

    public static Pose2d getTurretPose(Pose2d robotPose, double turretRelativeAngleRad) {
        double robotHeading = robotPose.heading.toDouble();
        double xOffset = -Math.cos(robotHeading) * turretParams.offsetFromCenter;
        double yOffset = -Math.sin(robotHeading) * turretParams.offsetFromCenter;
        return new Pose2d(robotPose.position.x + xOffset, robotPose.position.y + yOffset, robotHeading + turretRelativeAngleRad);
    }
    public static Pose2d getRobotPose(Pose2d turretPose, int turretPosition) {
        double relTurretAngleRad = Turret.getTurretRelativeAngleRad(turretPosition);
        double robotHeading = turretPose.heading.toDouble() - relTurretAngleRad;
        if(robotHeading > Math.PI)
            robotHeading -= Math.PI * 2;
        Vector2d robotTurretVec = new Vector2d(turretParams.offsetFromCenter * Math.cos(robotHeading), turretParams.offsetFromCenter * Math.sin(robotHeading));
        return new Pose2d(turretPose.position.x + robotTurretVec.x, turretPose.position.y + robotTurretVec.y, robotHeading);
    }
}
