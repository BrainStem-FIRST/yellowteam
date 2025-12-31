package org.firstinspires.ftc.teamcode.utils.pidDrive;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.Arrays;

public class UtilFunctions {
    public static Pose2d createPose(double[] pose) {
        if (pose.length < 3)
            throw new IllegalArgumentException("cannot call createPose on " + Arrays.toString(pose) + " - must contain at least 3 elements");
        return new Pose2d(pose[0], pose[1], Math.toRadians(pose[2]));
    }
}
