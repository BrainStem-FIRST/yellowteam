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
        this(pose, new Tolerance(Tolerance.defaultParams.xTol, Tolerance.defaultParams.yTol, Tolerance.defaultParams.headingDegTol), new PathParams());
    }
    public Waypoint(Pose2d pose, Tolerance tolerance) {
        this(pose, tolerance, new PathParams());
    }
    public Waypoint(Pose2d pose, PathParams pathParams) {
        this(pose, new Tolerance(Tolerance.defaultParams.xTol, Tolerance.defaultParams.yTol, Tolerance.defaultParams.headingDegTol), pathParams);
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


    @Override
    @NonNull
    public String toString() {
       return "x: " + x() + ", y: " + y() + ", heading: " + MathUtils.format2(headingDeg());
    }
}
