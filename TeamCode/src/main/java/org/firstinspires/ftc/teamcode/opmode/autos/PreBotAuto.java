package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.AutoCommands;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@Autonomous(name="Test Auto", group="Robot")
public class PreBotAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime autoTime = new ElapsedTime();
        autoTime.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d startPose = new Pose2d(-60, 20, Math.toRadians(0));

        BrainSTEMRobot robot = new BrainSTEMRobot(Alliance.BLUE, telemetry, hardwareMap, startPose);
        MecanumDrive drive = robot.drive;
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);

        Pose2d approachFirstLine = new Pose2d(-35,35, Math.toRadians(90));
        Pose2d approachSecondLine = new Pose2d(28, 35, Math.toRadians(90));
        Pose2d shootingPosition = new Pose2d(-60, 20, Math.toRadians(0));

        TrajectoryActionBuilder firstCycle = drive.actionBuilder(startPose)
                .splineTo(approachFirstLine.position, approachFirstLine.heading)
                .lineToY(50)
                .setReversed(true)
                .splineToLinearHeading(shootingPosition, Math.toRadians(0));

        TrajectoryActionBuilder secondCycle = drive.actionBuilder(shootingPosition)
                .splineTo(approachSecondLine.position, approachSecondLine.heading)
                .lineToY(50)
                .setReversed(true)
                .splineToLinearHeading(shootingPosition, Math.toRadians(0));

        Action firstMove = firstCycle.build();
        Action secondMove = secondCycle.build();

//        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        telemetryPacket -> {
                            robot.update();
                            return true;
                        },
                        new SequentialAction(
                            telemetryPacket -> {
                                robot.turret.turretState = Turret.TurretState.TRACKING;
                                return false;
                            },
                            firstMove,
                            telemetryPacket -> {
                                PoseStorage.currentPose = robot.drive.localizer.getPose();
                                return false;
                            }
                        )
                )
        );
    }
}