package org.firstinspires.ftc.teamcode.utils.pidDrive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.Arrays;

public class UtilFunctions {
    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
