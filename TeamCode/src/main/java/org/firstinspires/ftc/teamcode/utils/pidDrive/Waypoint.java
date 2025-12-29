package org.firstinspires.ftc.teamcode.utils.pidDrive;


import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

public class Waypoint {
    public Pose2d pose;
    public Tolerance tolerance;
    public PathParams params;
    private double distToNextWaypoint;
    public Waypoint(Pose2d pose) {
        this(pose, new Tolerance(Tolerance.defaultParams.xTol, Tolerance.defaultParams.yTol, Tolerance.defaultParams.headingRadTol), new PathParams());
    }
    public Waypoint(Pose2d pose, Tolerance tolerance) {
        this(pose, tolerance, new PathParams());
    }
    public Waypoint(Pose2d pose, PathParams pathParams) {
        this(pose, new Tolerance(Tolerance.defaultParams.xTol, Tolerance.defaultParams.yTol, Tolerance.defaultParams.headingRadTol), pathParams);
    }
   public Waypoint(Pose2d pose, Tolerance tolerance, PathParams pathParams) {
        this.pose = pose;
        this.tolerance = tolerance;
        this.params = pathParams;
    }
    public double x() {
        return pose.position.x;
    }
    public double y() {
        return pose.position.y;
    }
    public double headingDeg() {
        return Math.toDegrees(pose.heading.toDouble());
    }
    public double headingRad() {
       return pose.heading.toDouble();
    }

    public void setDistToNextWaypoint(double dist) {
        distToNextWaypoint = dist;
    }
    public double getDistToNextWaypoint() {
        return distToNextWaypoint;
    }
    public Waypoint setMaxTime(double t) {
        params.maxTime = t;
        return this;
    }
    public Waypoint setTol(Tolerance tol) {
        tolerance = tol;
        return this;
    }
    public Waypoint setPassPosition() {
        params.passPosition = true;
        return this;
    }
    public Waypoint setSlowDown(double s) {
        params.slowDownPercent = s;
        return this;
    }
    public Waypoint setHeadingLerp(PathParams.HeadingLerpType h) {
        params.headingLerpType = h;
        return this;
    }
    public Waypoint setMaxPower(double m) {
        params.maxLinearPower = m;
        return this;
    }

    @Override
    @NonNull
    public String toString() {
       return "x: " + x() + ", y: " + y() + ", heading: " + MathUtils.format2(headingDeg());
    }
}
