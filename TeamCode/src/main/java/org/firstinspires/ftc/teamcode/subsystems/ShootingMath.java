package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Turret.TURRET_PARAMS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.math.Vec;

@Config
public class ShootingMath {
    // stores all parameters of the shooter/hood/turret system
    public static class ShooterSystemParams {
        public double flywheelHeightMeters = 0.2413;
        public double targetHeightInches = 48;
        public double flywheelOffsetFromTurretInches = 2.4783465;
        public double flywheelRadiusMeters = 0.05;
        public double ballRadiusMeters = 0.064;
        public double gripCoefficient = 1; // actual exit velocity / theoretical exit velocity
        public double shooterMotorTicksPerRev = 28;
        public double flywheelTicksPerRev = shooterMotorTicksPerRev * 30 / 22; // pulley ratio is 30:22

        // slope and y intercept of distance-exit speed regression
        public double exitSpeedSlope = 4.53882;
        public double exitSpeedYIntercept = 800.29968;
        public double flywheelSpeedRelativeVelocityMultiplier = 1;
    }
    public static class HoodSystemParams {
        public double restingDistanceMm = 82;
        public double hoodPivotAngleOffsetFromHoodExitAngleDeg = 7.8;
        public double servoRangeMm = 30;
        public double minAngleDeg = 20, maxAngleDeg = 50;
        public double hoodAngleRelativeVelocityMultiplier = 1;
    }
    public static class TurretSystemParams {
        // the ball exit position must be traveling at a speed (in/sec) greater than this to account for its relative velocity
        public double predictVelocityExitSpeedThresholdInchesPerSec = 20;
        public double predictVelocityMultiplier = 1;
        public double maxAngleDeg = 90, minAngleDeg = -90;
    }
    public static ShooterSystemParams shooterSystemParams = new ShooterSystemParams();
    public static HoodSystemParams hoodSystemParams = new HoodSystemParams();
    public static TurretSystemParams turretSystemParams = new TurretSystemParams();
    public static boolean enableRelativeVelocity = false;

    // what this is used for as of 11/23
    // calculating distance from exit position to goal in updateShooterSystem
    // calculating current and future exit position pose for turret relative velocity correction
    // dashboard field telemetry and raw subsystem testing
    public static Vector2d calculateExitPositionInches(Pose2d robotPose, int turretEncoder, double ballExitAngleRad) {
        double hoodAngleRad = Math.PI * 0.5 - ballExitAngleRad;
        Pose2d turretPose = Turret.getTurretPose(robotPose, turretEncoder);
        double shooterCombinedRadiusInches = (shooterSystemParams.flywheelRadiusMeters + shooterSystemParams.ballRadiusMeters) / 0.0254;
        double offsetFromTurretInches = shooterSystemParams.flywheelOffsetFromTurretInches - Math.cos(hoodAngleRad) * shooterCombinedRadiusInches;

        return new Vector2d(
                turretPose.position.x + offsetFromTurretInches * Math.cos(turretPose.heading.toDouble()),
                turretPose.position.y + offsetFromTurretInches * Math.sin(turretPose.heading.toDouble())
        );
    }

    // what this is used for as of 11/23
    // calculating hood angle
    // raw subsystem testing telemetry
    public static double calculateExitHeightMeters(double ballExitAngleRad) {
        double hoodAngleRad = Math.PI * 0.5 - ballExitAngleRad;
        return shooterSystemParams.flywheelHeightMeters + (shooterSystemParams.flywheelRadiusMeters + shooterSystemParams.ballRadiusMeters) * Math.sin(hoodAngleRad);
    }

    // finds exit speed of ball (meters per sec) if flywheel is spinning at ticksPerSec
    public static double ticksPerSecToExitSpeedMps(double ticksPerSec) {
        double revPerSec = ticksPerSec / shooterSystemParams.flywheelTicksPerRev;
        double angularVel = revPerSec * 2 * Math.PI;
        return angularVel * shooterSystemParams.flywheelRadiusMeters * shooterSystemParams.gripCoefficient;
    }

    // finds required speed of flywheel (encoder ticks per sec) to shoot the ball at a speed of mps
    public static double mpsToTicksPerSec(double mps) {
        double wheelCircumference = 2 * Math.PI * shooterSystemParams.flywheelRadiusMeters;
        double revPerSec = mps / wheelCircumference;
        double idealTicksPerSec = revPerSec * shooterSystemParams.flywheelTicksPerRev;
        return idealTicksPerSec  / shooterSystemParams.gripCoefficient;
    }

    // calculates the component of robot velocity that is towards the goal
    // TODO - USE BALL EXIT VELOCITY INSTEAD OF ROBOT VELOCITY
    public static double calculateSpeedTowardGoalMps(Pose2d targetPose, Vector2d ballExitPosition, OdoInfo mostRecentVelocity) {
        double dx = targetPose.position.x - ballExitPosition.x;
        double dy = targetPose.position.y - ballExitPosition.y;

        double distanceFromGoal = Math.hypot(dx, dy);
        double ux = dx / distanceFromGoal;
        double uy = dy / distanceFromGoal;

        double vx = mostRecentVelocity.x;
        double vy = mostRecentVelocity.y;

        return -(vx * ux + vy * uy) * 0.0254;
    }

    // calculate flywheel speed (encoder ticks per sec) given a bunch of shooterSystemParams
    // can specify whether to enable or disable relative velocity prediction w/ static constant above
    // TODO - USE BALL EXIT VELOCITY INSTEAD OF ROBOT VELOCITY
    public static double calculateFlywheelSpeedTicksPerSec(Pose2d targetPose, double inchesFromGoal, Vector2d ballExitPosition, OdoInfo mostRecentVelocity) {
        double absoluteExitSpeedTicksPerSec = (shooterSystemParams.exitSpeedSlope * inchesFromGoal) + shooterSystemParams.exitSpeedYIntercept;
        if (!enableRelativeVelocity)
            return absoluteExitSpeedTicksPerSec;

        double absoluteExitSpeedMps = ticksPerSecToExitSpeedMps(absoluteExitSpeedTicksPerSec);
        // SHOULD return positive value
        double exitPositionSpeedTowardsGoalMps = ShootingMath.calculateSpeedTowardGoalMps(targetPose, ballExitPosition, mostRecentVelocity);
        double relativeExitSpeedMps = absoluteExitSpeedMps - (exitPositionSpeedTowardsGoalMps * shooterSystemParams.flywheelSpeedRelativeVelocityMultiplier);
        return mpsToTicksPerSec(relativeExitSpeedMps);
    }
    // calculates where the turret should point given a bunch of shooterSystemParams
    // can specify whether to enable or disable relative velocity prediction w/ static constant above
    public static double calculateTurretTargetAngleRad(Pose2d targetPose, Pose2d futureRobotPose, Vector2d currentExitPosition, Vector2d futureExitPosition, double ballExitSpeedMps) {
        Vec ballExitLinearVelocityInchesPerSec = new Vec(futureExitPosition.x - currentExitPosition.x, futureExitPosition.y - currentExitPosition.y);

        // use predicted turret position to account for turret lag
        Vec ballExitToGoal = new Vec(targetPose.position.x - futureExitPosition.x,
                targetPose.position.y - futureExitPosition.y).normalize();

        double targetAngleRad;
        // account for relative velocity
        if (enableRelativeVelocity && ballExitLinearVelocityInchesPerSec.mag() > turretSystemParams.predictVelocityExitSpeedThresholdInchesPerSec) {
            // find speed of ball relative to the ground (magnitude only)

            // find velocity of ball relative to the ground (direction and magnitude)
            Vec globalBallExitVelocityMps = ballExitToGoal.mult(ballExitSpeedMps);

            // find exit velocity in meters and apply empirical multiplier
            Vec exitVelocityMps = ballExitLinearVelocityInchesPerSec.mult(0.0254 * turretSystemParams.predictVelocityMultiplier);

            // velocity of ball relative to robot = velocity of ball relative to ground - velocity of robot relative to ground
            Vec relativeBallExitVelocityMps = globalBallExitVelocityMps.sub(exitVelocityMps);

            // find angle to shoot at relative velocity
            targetAngleRad = Math.atan2(relativeBallExitVelocityMps.y, relativeBallExitVelocityMps.x);
        }
        // find angle to shoot at without any relative velocity calculations
        else
            targetAngleRad = Math.atan2(ballExitToGoal.y, ballExitToGoal.x);

        double turretTargetAngleRad = targetAngleRad - futureRobotPose.heading.toDouble();
        // wrap between -pi, pi
        turretTargetAngleRad = Math.atan2(Math.sin(turretTargetAngleRad), Math.cos(turretTargetAngleRad));

        // mirrors the angle if the turret cannot reach it (visual cue)
        if (turretTargetAngleRad > Math.toRadians(turretSystemParams.maxAngleDeg))
            turretTargetAngleRad = Math.PI - turretTargetAngleRad;
        else if (turretTargetAngleRad < Math.toRadians(turretSystemParams.minAngleDeg))
            turretTargetAngleRad = -Math.PI - turretTargetAngleRad;

        return turretTargetAngleRad;
    }

    // calculates desired exit angle (radians) for ball given a bunch of shooterSystemParams
    // can specify whether to enable or disable relative velocity prediction w/ static constant above
    // TODO - USE BALL EXIT VELOCITY INSTEAD OF ROBOT VELOCITY
    public static double calculateBallExitAngleRad(Pose2d targetPose, Vector2d ballExitPosition, double distance, double ballExitSpeedMps, double prevBallExitAngleRad, OdoInfo mostRecentVelocity) {
        // get the EXACT exit height of the ball (depends on hood position)
        double exitHeightMeters = ShootingMath.calculateExitHeightMeters(prevBallExitAngleRad);
        double heightToTargetMeters = (shooterSystemParams.targetHeightInches * 0.0254 - exitHeightMeters);

        // Physics formula rearranged for angle: tan(θ) = (v² ± √(v⁴ - g(gx² + 2yv²))) / (gx)
        double g = 9.81;
        double x = distance * 0.0254; // convert inches to meters
        double y = heightToTargetMeters; // convert inches to meters

        double v = ballExitSpeedMps;
        if (enableRelativeVelocity) {
            double exitPositionSpeedTowardsGoalMps = ShootingMath.calculateSpeedTowardGoalMps(targetPose, ballExitPosition, mostRecentVelocity);
            v += exitPositionSpeedTowardsGoalMps * hoodSystemParams.hoodAngleRelativeVelocityMultiplier;
        }
        double discriminant = v*v*v*v - g*(g*x*x + 2*y*v*v);
        if (discriminant <= 0)
            return Math.toRadians(40);

        double tanTheta = (v*v - Math.sqrt(discriminant)) / (g * x);
        return Math.atan(tanTheta);
    }

    // calculates the position to put the linear actuators to on the shooter hood
    public static double calculateHoodServoPosition(double ballExitAngleRadians, Telemetry telemetry) {
        double hoodAngleFromXAxisRadians = Math.PI * 0.5 - ballExitAngleRadians;
        double hoodExitAngleDeg = Range.clip(Math.toDegrees(hoodAngleFromXAxisRadians), hoodSystemParams.minAngleDeg, hoodSystemParams.maxAngleDeg);
        double hoodPivotAngleDeg = hoodExitAngleDeg + hoodSystemParams.hoodPivotAngleOffsetFromHoodExitAngleDeg;
        double totalLinearDistanceMm = -0.00125315 * Math.pow(hoodPivotAngleDeg, 2) + 0.858968 * hoodPivotAngleDeg + 63.03978;
        double linearDistanceToExtendMm = totalLinearDistanceMm - hoodSystemParams.restingDistanceMm;
        if (telemetry != null) {
            telemetry.addLine("HOOD SERVO POS CALCULATION");
            telemetry.addData("ball exit angle rad", ballExitAngleRadians);
            telemetry.addData("hood Angle from x", hoodAngleFromXAxisRadians);
            telemetry.addData("total linear distance mm", totalLinearDistanceMm);
        }
        return linearDistanceToExtendMm / hoodSystemParams.servoRangeMm;
    }
}
