package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

public class PIDDriveTest extends LinearOpMode {
    public static double tolX = 1.5, tolY = 1.5, tolH = 3;
    public static double tolX2 = 1.5, tolY2 = 1.5, tolH2 = 3;
    public static double maxTime = 100;
    public static double x0 = 0, y0 = 0, h0 = Math.toRadians(180);
    public static double x1 = -20, y1 = 0, h1 = Math.toRadians(180);
    public static double x2 = -24, y2 = 4, h2 = Math.toRadians(90);
    public static double x3 = -24, y3 = 24, h3 = Math.toRadians(90);
    public static boolean passPosition1 = false;
    public static double slowDownPercent1 = 0.4, slowDownPercent2 = 0.4;
    public static double maxSpeed = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(x0, y0, h0));

        Waypoint w1 = new Waypoint(new Pose2d(x1, y1, h1), new Tolerance(tolX, tolY, tolH));
        w1.params.maxTime = maxTime;
        w1.params.passPosition = passPosition1;
        w1.params.slowDownPercent = slowDownPercent1;
        w1.params.maxLinearPower = maxSpeed;
        Waypoint w2 = new Waypoint(new Pose2d(x2, y2, h2), new Tolerance(tolX2, tolY2, tolH2));
        w2.params.maxTime = maxTime;
        w2.params.maxLinearPower = maxSpeed;
        w2.params.slowDownPercent = slowDownPercent2;
        Waypoint w3 = new Waypoint(new Pose2d(x3, y3, h3), new Tolerance(tolX2, tolY2, tolH2));
        w3.params.maxTime = maxTime;
        w3.params.maxLinearPower = maxSpeed;

        DrivePath drivePath = new DrivePath(drive, telemetry, w1, w2, w3);

        waitForStart();
        Actions.runBlocking(drivePath);
    }
}
