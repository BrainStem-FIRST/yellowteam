package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Turret.turretParams;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.math.Vector3d;

@Config
public class ShootingMath {
    // stores all parameters of the shooter/hood/turret system
    public static class ShooterSystemParams {
        public double flywheelHeightMeters = 0.2413;
        public double flywheelOffsetFromTurretInches = 2.4783465;
        public double flywheelRadiusMeters = 0.0445;
        public double ballRadiusMeters = 0.064;
        public double shooterMotorTicksPerRev = 28;
    }
    public static class HoodSystemParams {
        public double restingDistanceMm = 82;
        public double hoodPivotAngleOffsetFromHoodExitAngleDeg = 7.8;
        public double servoRangeMm = 30;
        public double minAngleDeg = 15, maxAngleDeg = 55;
        public double highArcHoodAngleDegEstimation = 20, lowArcHoodAngleDegEstimation = 45;
    }
    public static class TurretSystemParams {
        public double maxAngleDeg = 90, minAngleDeg = -90;
    }
    public static ShooterSystemParams shooterSystemParams = new ShooterSystemParams();
    public static HoodSystemParams hoodSystemParams = new HoodSystemParams();
    public static TurretSystemParams turretSystemParams = new TurretSystemParams();

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
    public static double approximateExitHeightM(boolean useHighArc) {
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

    public static Vector3d calculateActualTargetExitVel(double targetBallTravelAngle, double ballExitAngleRad, double targetVelMps, Vector2d robotVelAtExitPosMps) {
        Vector3d ballTravelDir = new Vector3d(Math.cos(targetBallTravelAngle),0, Math.sin(targetBallTravelAngle));
        Vector2d shootingAngle = new Vector2d(Math.cos(ballExitAngleRad), Math.sin(ballExitAngleRad));
        Vector3d absoluteTargetVelocity = ballTravelDir.times(shootingAngle.x).plus(Vector3d.j.times(shootingAngle.y));
        absoluteTargetVelocity = absoluteTargetVelocity.times(targetVelMps);

        return absoluteTargetVelocity.minus(new Vector3d(robotVelAtExitPosMps.x, 0, robotVelAtExitPosMps.y));
    }

    // calculates desired exit angle (radians) for ball given a bunch of shooterSystemParams
    // can specify whether to enable or disable relative velocity prediction w/ static constant above
    // TODO - USE BALL EXIT VELOCITY INSTEAD OF ROBOT VELOCITY
    public static double calculateBallExitAngleRad(boolean useHighArc, double y, double distanceInches, double ballExitSpeedMps) {

        // Physics formula rearranged for angle: tan(θ) = (v² ± √(v⁴ - g(gx² + 2yv²))) / (gx)
        double g = 9.81;
        double x = distanceInches * 0.0254; // convert inches to meters

        double v = ballExitSpeedMps;
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
