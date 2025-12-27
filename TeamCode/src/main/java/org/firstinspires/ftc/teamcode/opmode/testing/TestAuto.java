package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

@Config
@Autonomous(name="test auto")
public class TestAuto extends LinearOpMode {
    public static double[] pose0 = {0, 0, 0};
    public static double[] shootPose = {-48, -30, 90};
    public static double[] collect1Pose = {-48, 64, 90};
    public static double[] preCollect2Pose = {-55, -10, 90};
    public static double[] collect2Pose = {-55, 64, 90};
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d begin = MathUtils.createPose(pose0);
        Pose2d shoot = MathUtils.createPose(shootPose);
        Pose2d collect1 = MathUtils.createPose(collect1Pose);
        Pose2d preCollect2 = MathUtils.createPose(preCollect2Pose);
        Pose2d collect2 = MathUtils.createPose(collect2Pose);

        MecanumDrive drive = new MecanumDrive(hardwareMap, begin);

        Waypoint collect2w1 = new Waypoint(collect2);
        collect2w1.params.slowDownPercent = 0.5;

        Action autoAction = new SequentialAction(
                new DrivePath(hardwareMap, drive, telemetry, new Waypoint(shoot)),
                new DrivePath(hardwareMap, drive, telemetry, new Waypoint(collect1)),
                new DrivePath(hardwareMap, drive, telemetry, new Waypoint(shoot)),
                new DrivePath(hardwareMap, drive, telemetry, collect2w1, new Waypoint(collect2)),
                new DrivePath(hardwareMap, drive, telemetry, new Waypoint(shoot))
        );
        waitForStart();
        Actions.runBlocking(autoAction);
    }
}
