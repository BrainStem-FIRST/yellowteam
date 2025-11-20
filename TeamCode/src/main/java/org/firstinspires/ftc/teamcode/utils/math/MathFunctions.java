package org.firstinspires.ftc.teamcode.utils.math;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class MathFunctions {

    public static double kP = 0.75;
    public static double kI = 0.0;
    public static double kD = 0.05;

    private PIDController pid;

    public MathFunctions() {
        pid = new PIDController(kP, kI, kD);
        pid.setOutputBounds(-1, 1);
    }

    public void updatePIDFromDashboard() {
        pid.setPIDValues(kP, kI, kD);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double getTurnPower(Pose2d robotPose, Pose2d targetPose) {
        updatePIDFromDashboard();

        double dx = targetPose.position.x - robotPose.position.x;
        double dy = targetPose.position.y - robotPose.position.y;

        double targetHeading = Math.atan2(dy, dx);

        double error = normalizeAngle(targetHeading - robotPose.heading.toDouble());

        return pid.updateWithError(error);
    }

    public double turretToYaw(Pose2d robotPose, Pose2d goalPose) {
        double dx = goalPose.position.x - robotPose.position.x;
        double dy = goalPose.position.y - robotPose.position.y;

        double targetHeading = Math.atan2(dy, dx);
        double currentHeading = robotPose.heading.toDouble();
        double angleDiff = normalizeAngle(targetHeading - currentHeading);

        return Math.toDegrees(angleDiff);
    }

    public double hoodToPitch(Pose2d robotPose, Pose2d goalPose) {
        return 0;
    }

    public double calculateInitialVelocity() {
        return 0;
    }
}
