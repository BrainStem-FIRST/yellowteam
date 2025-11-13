package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(0, 0, Math.toRadians(0));
    public static int currentTurretEncoder = 0;
}