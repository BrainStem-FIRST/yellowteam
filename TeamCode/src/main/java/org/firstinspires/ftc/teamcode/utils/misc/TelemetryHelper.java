package org.firstinspires.ftc.teamcode.utils.misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class TelemetryHelper {
    public static String[] colors = { "red", "green", "blue" };
    public static Integer[] strokeWidths = { 1, 1, 1 };
    public static Integer[] radii = { 3, 3, 3 };
    public static int numPosesToShow = 2;
    public static double strokeAlpha = 0.5;
    public static double fieldRotation = 0;

    public static void sendRobotPoses(Pose2d ...poses) {
        TelemetryPacket packet = new TelemetryPacket();
        addRobotPoseToCanvas(packet.fieldOverlay(), poses);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
    public static void addRobotPoseToCanvas(Canvas fieldOverlay, Pose2d...poses) {
        fieldOverlay.setRotation(Math.toRadians(fieldRotation)); // rotate 90deg clockwise

        for (int i=0; i<Math.min(numPosesToShow, poses.length); i++) {
            Pose2d pose = poses[i];
            fieldOverlay.setAlpha(strokeAlpha);
            fieldOverlay.setStrokeWidth(strokeWidths[i % strokeWidths.length]);
            fieldOverlay.setStroke(colors[i % colors.length]);
            double x = pose.position.x, y = pose.position.y, heading = pose.heading.toDouble();
            fieldOverlay.strokeCircle(x, y, radii[i]);
            fieldOverlay.strokeLine(x, y, x + radii[i] * Math.cos(heading), y + radii[i] * Math.sin(heading));
        }
    }
}
