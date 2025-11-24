package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Shooter.HOOD_PARAMS;
import static org.firstinspires.ftc.teamcode.subsystems.Shooter.SHOOTER_PARAMS;
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
    public static boolean enableRelativeVelocity = false;

    // what this is used for as of 11/23
    // calculating distance from exit position to goal in updateShooterSystem
    // calculating current and future exit position pose for turret relative velocity correction
    // dashboard field telemetry and raw subsystem testing
    public static Vector2d calculateExitPositionInches(Pose2d robotPose, int turretEncoder, double ballExitAngleRad) {
        double hoodAngleRad = Math.PI * 0.5 - ballExitAngleRad;
        Pose2d turretPose = Turret.getTurretPose(robotPose, turretEncoder);
        double shooterCombinedRadiusInches = (SHOOTER_PARAMS.FLYWHEEL_RADIUS_METERS + SHOOTER_PARAMS.BALL_RADIUS_METERS) / 0.0254;
        double offsetFromTurretInches = SHOOTER_PARAMS.FLYWHEEL_OFFSET_FROM_TURRET_INCHES - Math.cos(hoodAngleRad) * shooterCombinedRadiusInches;

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
        return SHOOTER_PARAMS.FLYWHEEL_HEIGHT_METERS + (SHOOTER_PARAMS.FLYWHEEL_RADIUS_METERS + SHOOTER_PARAMS.BALL_RADIUS_METERS) * Math.sin(hoodAngleRad);
    }

    // finds exit speed of ball (meters per sec) if flywheel is spinning at ticksPerSec
    public static double ticksPerSecToExitSpeedMps(double ticksPerSec) {
        double revPerSec = ticksPerSec / SHOOTER_PARAMS.FLYWHEEL_TICKS_PER_REV;
        double angularVel = revPerSec * 2 * Math.PI;
        return angularVel * SHOOTER_PARAMS.FLYWHEEL_RADIUS_METERS * SHOOTER_PARAMS.GRIP_COEFFICIENT;
    }

    // finds required speed of flywheel (encoder ticks per sec) to shoot the ball at a speed of mps
    public static double mpsToTicksPerSec(double mps) {
        double wheelCircumference = 2 * Math.PI * SHOOTER_PARAMS.FLYWHEEL_RADIUS_METERS;
        double revPerSec = mps / wheelCircumference;
        double idealTicksPerSec = revPerSec * SHOOTER_PARAMS.FLYWHEEL_TICKS_PER_REV;
        return idealTicksPerSec  / SHOOTER_PARAMS.GRIP_COEFFICIENT;
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

    // calculate flywheel speed (encoder ticks per sec) given a bunch of params
    // can specify whether to enable or disable relative velocity prediction w/ static constant above
    // TODO - USE BALL EXIT VELOCITY INSTEAD OF ROBOT VELOCITY
    public static double calculateFlywheelSpeedTicksPerSec(Pose2d targetPose, double inchesFromGoal, Vector2d ballExitPosition, OdoInfo mostRecentVelocity) {
        double absoluteExitSpeedTicksPerSec = (SHOOTER_PARAMS.SLOPE_CLOSE_VALUE * inchesFromGoal) + SHOOTER_PARAMS.B_CLOSE_VALUE;
        if (!enableRelativeVelocity)
            return absoluteExitSpeedTicksPerSec;

        double absoluteExitSpeedMps = ticksPerSecToExitSpeedMps(absoluteExitSpeedTicksPerSec);
        // SHOULD return positive value
        double exitPositionSpeedTowardsGoalMps = ShootingMath.calculateSpeedTowardGoalMps(targetPose, ballExitPosition, mostRecentVelocity);
        double relativeExitSpeedMps = absoluteExitSpeedMps - (exitPositionSpeedTowardsGoalMps * SHOOTER_PARAMS.RELATIVE_VELOCITY_CORRECTION);
        return mpsToTicksPerSec(relativeExitSpeedMps);
    }
    // calculates where the turret should point given a bunch of params
    // can specify whether to enable or disable relative velocity prediction w/ static constant above
    public static double calculateTurretTargetAngleRad(Pose2d targetPose, Pose2d futureRobotPose, Vector2d currentExitPosition, Vector2d futureExitPosition, double ballExitSpeedMps) {
        double turretMax = Math.toRadians(90);
        double turretMin = Math.toRadians(-90);

//        telemetry.addData("currentRobotPose", currentRobotPose);
//        telemetry.addData("targetPose", targetPose);
//        telemetry.addData("futureRobotPose", futureRobotPose);
//        telemetry.addData("currentTurretPose", currentTurretPose);
//        telemetry.addData("futureTurretPose", futureTurretPose);
        Vec ballExitLinearVelocityInchesPerSec = new Vec(futureExitPosition.x - currentExitPosition.x, futureExitPosition.y - currentExitPosition.y);

        // predict turret position to account for turret lag
        Vec ballExitToGoal = new Vec(targetPose.position.x - futureExitPosition.x,
                targetPose.position.y - futureExitPosition.y).normalize();

        double targetAngleRad;
        // only account for robot velocity if it is significant
        if (ballExitLinearVelocityInchesPerSec.mag() > TURRET_PARAMS.predictVelocityExitSpeedThresholdInchesPerSec && enableRelativeVelocity) {
            // find speed of ball relative to the ground (magnitude only)

            // find velocity of ball relative to the ground (direction and magnitude)
            Vec globalBallExitVelocityMps = ballExitToGoal.mult(ballExitSpeedMps);

            // find exit velocity in meters and apply empirical multiplier
            Vec exitVelocityMps = ballExitLinearVelocityInchesPerSec.mult(0.0254 * TURRET_PARAMS.predictVelocityMultiplier);

            // velocity of ball relative to robot = velocity of ball relative to ground - velocity of robot relative to ground
            Vec relativeBallExitVelocityMps = globalBallExitVelocityMps.sub(exitVelocityMps);

            // find angle to shoot at relative velocity
            targetAngleRad = Math.atan2(relativeBallExitVelocityMps.y, relativeBallExitVelocityMps.x);
        } else
            // find angle to shoot at normally
            targetAngleRad = Math.atan2(ballExitToGoal.y, ballExitToGoal.x);

        double turretTargetAngleRad = targetAngleRad - futureRobotPose.heading.toDouble();
        // wrap between -pi, pi
        turretTargetAngleRad = Math.atan2(Math.sin(turretTargetAngleRad), Math.cos(turretTargetAngleRad));

        // just mirror the angle if the turret cannot reach it for visual cue
        if (turretTargetAngleRad > turretMax)
            turretTargetAngleRad = Math.PI - turretTargetAngleRad;
        else if (turretTargetAngleRad < turretMin)
            turretTargetAngleRad = -Math.PI - turretTargetAngleRad;

        return turretTargetAngleRad;
    }

    // calculates desired exit angle (radians) for ball given a bunch of params
    // can specify whether to enable or disable relative velocity prediction w/ static constant above
    // TODO - USE BALL EXIT VELOCITY INSTEAD OF ROBOT VELOCITY
    public static double calculateBallExitAngleRad(Pose2d targetPose, Vector2d ballExitPosition, double distance, double ballExitSpeedMps, double prevBallExitAngleRad, OdoInfo mostRecentVelocity) {
        // get the EXACT exit height of the ball (depends on hood position)
        double exitHeightMeters = ShootingMath.calculateExitHeightMeters(prevBallExitAngleRad);
        double heightToTargetMeters = (SHOOTER_PARAMS.TARGET_HEIGHT_INCHES * 0.0254 - exitHeightMeters);

        // Physics formula rearranged for angle: tan(θ) = (v² ± √(v⁴ - g(gx² + 2yv²))) / (gx)
        double g = 9.81;
        double x = distance * 0.0254; // convert inches to meters
        double y = heightToTargetMeters; // convert inches to meters

        double v = ballExitSpeedMps;
        if (enableRelativeVelocity) {
            double exitPositionSpeedTowardsGoalMps = ShootingMath.calculateSpeedTowardGoalMps(targetPose, ballExitPosition, mostRecentVelocity);
            v += exitPositionSpeedTowardsGoalMps * SHOOTER_PARAMS.RELATIVE_VELOCITY_CORRECTION;
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
        double hoodExitAngleDeg = Range.clip(Math.toDegrees(hoodAngleFromXAxisRadians), Shooter.HoodParams.minAngleDeg, Shooter.HoodParams.maxAngleDeg);
        double hoodPivotAngleDeg = hoodExitAngleDeg + HOOD_PARAMS.hoodPivotAngleOffsetFromHoodExitAngleDeg;
        double totalLinearDistanceMm = -0.00125315 * Math.pow(hoodPivotAngleDeg, 2) + 0.858968 * hoodPivotAngleDeg + 63.03978;
        double linearDistanceToExtendMm = totalLinearDistanceMm - HOOD_PARAMS.restingDistanceMm;
        if (telemetry != null) {
            telemetry.addLine("HOOD SERVO POS CALCULATION");
            telemetry.addData("ball exit angle rad", ballExitAngleRadians);
            telemetry.addData("hood Angle from x", hoodAngleFromXAxisRadians);
            telemetry.addData("total linear distance mm", totalLinearDistanceMm);
        }
        return linearDistanceToExtendMm / HOOD_PARAMS.servoRangeMm;
    }
}
