package org.firstinspires.ftc.teamcode.utils.autoHelpers;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

import java.util.function.BooleanSupplier;

public class RRTolerance implements BooleanSupplier {
    private final PinpointLocalizer odo;
    public final Pose2d targetPose;
    public final double tolX, tolY, headingRadTol;
    public RRTolerance(PinpointLocalizer odo, Pose2d targetPose, double tolX, double tolY, double headingRadTol) {
        this.odo = odo;
        this.targetPose = targetPose;
        this.tolX = tolX;
        this.tolY = tolY;
        this.headingRadTol = headingRadTol;
    }
    public RRTolerance(PinpointLocalizer odo, Pose2d targetPose, double distTol, double headingRadTol) {
        this(odo, targetPose, distTol, distTol, headingRadTol);
    }
    @Override
    public boolean getAsBoolean() {
        Pose2d pose = odo.getPose();
        double xError = pose.position.x - targetPose.position.x;
        double yError = pose.position.y - targetPose.position.y;
        double headingErrorRad = pose.heading.toDouble() - targetPose.heading.toDouble();
        return Math.abs(xError) < tolX && Math.abs(yError) < tolY && Math.abs(headingErrorRad) < headingRadTol;
    }
}
