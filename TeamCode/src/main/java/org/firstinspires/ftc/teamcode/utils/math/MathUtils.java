package org.firstinspires.ftc.teamcode.utils.math;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;
import java.util.Arrays;

// helps for concise, easy printing to telemetry
public class MathUtils {
    public static double correctRad(double rad) {
        while (rad < 0)
            rad += 2 * Math.PI;
        while (rad >= 2 * Math.PI)
            rad -= 2 * Math.PI;
        return rad;
    }
    public static double correctDeg(double deg) {
        while (deg < 0)
            deg += 360;
        while (deg >= 360)
            deg -= 360;
        return deg;
    }
    public static String format1(Number num) {
        return format(num, 1);
    }
    public static String format2(Number num) {
        return format(num, 2);
    }
    public static String format3(Number num) { return format(num, 3); }
    public static String format(Number num, int decimalPlaces) {
        StringBuilder decimals = new StringBuilder();
        for (int i=0; i<decimalPlaces; i++)
            decimals.append("#");
        DecimalFormat customDf = new DecimalFormat("#." + decimals);
        return customDf.format(num);
    }
    public static String format2(double[] nums) {
        StringBuilder total = new StringBuilder();
        for (double num : nums)
            total.append(format2(num)).append(", ");
        return total.substring(0, total.length() - 2);
    }
    public static String format3(double[] nums) {
        StringBuilder total = new StringBuilder();
        for (double num : nums)
            total.append(format3(num)).append(", ");
        return total.substring(0, total.length() - 2);
    }
    public static String format2Pose(Pose2d pose) {
        return MathUtils.format2(pose.position.x) + " " + MathUtils.format2(pose.position.y) + MathUtils.format2(pose.heading.toDouble());
    }

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }


    public static Pose2d createPose(double[] pose) {
        if (pose.length < 3)
            throw new IllegalArgumentException("cannot call createPose on " + Arrays.toString(pose) + " - must contain at least 3 elements");
        return new Pose2d(pose[0], pose[1], Math.toRadians(pose[2]));
    }
}
