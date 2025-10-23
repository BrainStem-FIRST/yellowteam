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
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@Autonomous(name="Red Far Partner Auto", group="Robot")
public class RedFarPartnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime autoTime = new ElapsedTime();
        autoTime.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d startPose = new Pose2d(60, 20, Math.toRadians(180));

        BrainSTEMRobot robot = new BrainSTEMRobot(telemetry, hardwareMap, startPose);
        MecanumDrive drive = robot.drive;
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);

        Pose2d approachHP = new Pose2d(50,56, Math.toRadians(45));
        Pose2d atHP = new Pose2d(60, 56, Math.toRadians(45));
        Pose2d shootingPosition = new Pose2d(60, 20, Math.toRadians(180));
        Pose2d firstLine = new Pose2d(35, 35, Math.toRadians(90));

        TrajectoryActionBuilder hpShot = drive.actionBuilder(startPose)
//                .setTangent(Math.toRadians(90))
                .splineTo(approachHP.position, approachHP.heading)
//                .setTangent(Math.toRadians(90))
                .splineTo(approachHP.position, approachHP.heading)
                .splineTo(shootingPosition.position, shootingPosition.heading);

        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(shootingPosition)
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