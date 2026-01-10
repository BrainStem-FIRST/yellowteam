package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

public class TestAuto extends LinearOpMode {
    public static double[] begin = {-63.5, -40, 180};
    public static double[] shoot1 = {-15, -20, 180};
    public static double[] shoot2 = {-15, -20, -30};
    public static double[] collect1 = {-11, 48.5, -90};
    public static double[] preCollect2 = {14.5, -32, -90};
    public static double[] collect2 = {14.5, -52, -90};
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d begin = MathUtils.createPose(TestAuto.begin);
        Pose2d shoot1 = MathUtils.createPose(TestAuto.shoot1);
        Pose2d shoot2 = MathUtils.createPose(TestAuto.shoot2);
        Pose2d collect1 = MathUtils.createPose(TestAuto.collect1);
        Pose2d preCollect2 = MathUtils.createPose(TestAuto.preCollect2);
        Pose2d collect2 = MathUtils.createPose(TestAuto.collect2);

        BrainSTEMRobot robot = new BrainSTEMRobot(Alliance.BLUE, telemetry, hardwareMap, begin);
        MecanumDrive drive = new MecanumDrive(hardwareMap, begin);

        Waypoint collect2w1 = new Waypoint(preCollect2);
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);
        Action autoAction = new ParallelAction(
                new SequentialAction(
                        new ParallelAction(
                                new DrivePath(drive, telemetry, new Waypoint(shoot1)),
                                autoCommands.speedUpShooter()
                        ),
                        autoCommands.engageClutch(),
                        autoCommands.runIntake(),
                        autoCommands.flickerHalfUp(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                autoCommands.waitTillDoneShooting(1.5, 1)
                        ),
                        autoCommands.disengageClutch(),
                        new DrivePath(drive, telemetry, new Waypoint(collect1)),
                        autoCommands.intakeSlow(),
                        new ParallelAction(
                                new DrivePath(drive, telemetry, new Waypoint(shoot2)),
                                new SequentialAction(
                                        new SleepAction(0.15),
                                        autoCommands.stopIntake()
                                )
                        ),
                        autoCommands.runIntake(),
                        autoCommands.engageClutch(),
                        autoCommands.flickerHalfUp(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                autoCommands.waitTillDoneShooting(1.5, 1)
                        ),
                        new DrivePath(drive, telemetry, collect2w1, new Waypoint(collect2)),
                        autoCommands.intakeSlow(),
                        new ParallelAction(
                                new DrivePath(drive, telemetry, new Waypoint(shoot2)),
                                new SequentialAction(
                                        new SleepAction(0.15),
                                        autoCommands.stopIntake()
                                )
                        ),
                        telemetryPacket -> {robot.shooter.setBallsShot(0); return false;},
                        autoCommands.runIntake(),
                        autoCommands.engageClutch(),
                        autoCommands.flickerHalfUp(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                autoCommands.waitTillDoneShooting(1.5, 1)
                        ),
                        autoCommands.disengageClutch()
                ),
                autoCommands.updateRobot

        );
        waitForStart();
        Actions.runBlocking(autoAction);
    }
}
