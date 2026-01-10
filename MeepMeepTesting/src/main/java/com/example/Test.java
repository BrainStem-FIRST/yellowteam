package com.example;

public class Test {
    public static void main(String[] args) {
        /*
        Pose2d prevWaypointPose = curWaypointIndex == 0 ? startPose : getWaypoint(curWaypointIndex - 1).pose;
        curWaypointDirRad = Math.atan2(getCurWaypoint().pose.position.y - prevWaypointPose.position.y, getCurWaypoint().pose.position.x - prevWaypointPose.position.x);
        if (getCurParams().headingLerpType == PathParams.HeadingLerpType.REVERSE_TANGENT)
            curWaypointDirRad = MathUtils.angleNormDeltaRad(curWaypointDirRad + Math.PI);
         */
        double num = Math.atan2(24, -64);
        System.out.println(Math.toDegrees(num));
        System.out.println(Math.toDegrees(MathUtils.angleNormDeltaRad(num + Math.PI)));

    }
}
