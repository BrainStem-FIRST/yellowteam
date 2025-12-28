package org.firstinspires.ftc.teamcode.utils.math;

import com.acmerobotics.roadrunner.Pose2d;

import java.text.DecimalFormat;
import java.util.Arrays;

// helps for concise, easy printing to telemetry
public class MathUtils {
    public static double angleNormRad(double rad) {
        if(rad >= 0 && rad < Math.PI * 2)
            return rad;
        return (rad % (Math.PI * 2) + Math.PI * 2) % (Math.PI * 2);
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

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static String formatPose2(Pose2d pose) {
        if (pose == null)
            return "null";
        return format2(pose.position.x) + ", " + format2(pose.position.y) + " " + format2(Math.toDegrees(pose.heading.toDouble()));
    }
    public static Pose2d createPose(double[] pose) {
        if (pose.length < 3)
            throw new IllegalArgumentException("cannot call createPose on " + Arrays.toString(pose) + " - must contain at least 3 elements");
        return new Pose2d(pose[0], pose[1], Math.toRadians(pose[2]));
    }
}
