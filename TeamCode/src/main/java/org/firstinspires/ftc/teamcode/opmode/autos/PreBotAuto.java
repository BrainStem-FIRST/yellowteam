package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

@Autonomous(name="Test Auto", group="Robot")
public class PreBotAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime autoTime = new ElapsedTime();
        autoTime.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d startPose = new Pose2d(60, -20, Math.toRadians(180));

        BrainSTEMRobot robot = new BrainSTEMRobot(telemetry, hardwareMap, startPose);
        MecanumDrive drive = robot.drive;

        Pose2d approachLine = new Pose2d(35,-35, Math.toRadians(-90));
        Pose2d shootingPosition = new Pose2d(60, -20, Math.toRadians(180));

        TrajectoryActionBuilder firstCycle = drive.actionBuilder(startPose)
                .splineTo(approachLine.position, approachLine.heading)
                .lineToY(-50)
                .setReversed(true)
                .splineToLinearHeading(shootingPosition, Math.toRadians(0));

        Action testTrajectory = firstCycle.build();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        testTrajectory
                )
        );
        }
    }
