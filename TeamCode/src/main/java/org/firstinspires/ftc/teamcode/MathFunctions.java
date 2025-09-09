package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class MathFunctions {

    /// Odometry to find yaw → turret to yaw angle
    /// April tag to cross check yaw
    /// Distance to find pitch → hood angle
    /// April tag to cross check pitch
    /// Pitch, distance, height to solve for Velocity of Launch
    /// Apply correction based on (test values)
    /// Convert the velocity a power on motor

    public double turretToYaw(Pose2d robotPose, Pose2d goalPose) {
        double dx = goalPose.position.x - robotPose.position.x;
        double dy = goalPose.position.y - robotPose.position.y;

        double targetHeading = Math.atan2(dy, dx);
        double currentHeading = robotPose.heading.toDouble();
        double angleDiff = normalizeAngle(targetHeading - currentHeading);

        return Math.toDegrees(angleDiff);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double hoodToPitch(Pose2d robotPose, Pose2d goalPose) {
        return 0;
    }

    public double calculateInitialVelocity() {
        return 0;
    }
}