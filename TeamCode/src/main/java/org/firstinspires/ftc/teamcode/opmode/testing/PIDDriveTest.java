package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

@Autonomous(name="PIDDriveTest", group="Testing")
@Config
public class PIDDriveTest extends LinearOpMode {
    public static double skp = 0.05, ski = 0.001, skd = 0.004;
    public static double hkp = 0.6, hki = 0.01, hkd = 0;
    public static double tolX = 1, tolY = 1, tolH = 1;
    public static double maxTime = 100;
    public static double x1 = 24, y1 = 0, h1 = 0;
    public static double x2 = 48, y2 = 24, h2 = 0;
    public static boolean passPosition1 = true;
    public static double slowDownPercent1 = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Waypoint w1 = new Waypoint(new Pose2d(x1, y1, h1), new Tolerance(tolX, tolY, tolH), new PathParams(skp, ski, skd, hkp, hki, hkd));
        w1.params.maxTime = maxTime;
        w1.params.passPosition = passPosition1;
        w1.params.slowDownPercent = slowDownPercent1;
        Waypoint w2 = new Waypoint(new Pose2d(x2, y2, h2), new Tolerance(tolX, tolY, tolH), new PathParams(skp, ski, skd, hkp, hki, hkd));
        w2.params.maxTime = maxTime;

        DrivePath drivePath = new DrivePath(drive, telemetry, w1, w2);

        waitForStart();
        Actions.runBlocking(drivePath);
    }
}
