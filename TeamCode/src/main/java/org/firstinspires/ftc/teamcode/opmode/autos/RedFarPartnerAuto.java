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
import org.firstinspires.ftc.teamcode.utils.AutoCommands;

@Autonomous(name="Red Far Partner Auto", group="Robot")
public class RedFarPartnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime autoTime = new ElapsedTime();
        autoTime.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d startPose = new Pose2d(63.5, 18.5, Math.toRadians(180));

        BrainSTEMRobot robot = new BrainSTEMRobot(telemetry, hardwareMap, startPose);
        MecanumDrive drive = robot.drive;
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);

        // DRIVE POSITIONS
        Pose2d approachHP = new Pose2d(45,57, Math.toRadians(45));
        Pose2d atHP = new Pose2d(58, 64, Math.toRadians(45));
        Pose2d shootingPosition = new Pose2d(50, 10, Math.toRadians(180));
        Pose2d firstLine = new Pose2d(31, 36, Math.toRadians(90));

        // BUILDING FULL ACTIONS
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .splineToConstantHeading(shootingPosition.position, shootingPosition.heading);



        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(shootingPosition)
                .splineTo(firstLine.position, firstLine.heading)
                .lineToY(66)
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(shootingPosition, Math.toRadians(0));

        TrajectoryActionBuilder hpShot = drive.actionBuilder(shootingPosition)
                .splineTo(approachHP.position, approachHP.heading)
                .waitSeconds(2)
                .splineToLinearHeading(atHP, Math.toRadians(0));
//                .waitSeconds(2)
//                .setTangent(0)
//                .splineTo(shootingPosition.position, shootingPosition.heading);

        Action driveToShootingPose = moveOffWall.build();
        Action humanPlayerShots = hpShot.build();
        Action firstLineShots = firstLineShot.build();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(

            new ParallelAction(
                autoCommands.updateRobot,
                autoCommands.savePoseContinuously,

                new SequentialAction(
                    autoCommands.setRedAlliance(),

                    new ParallelAction(
//                        autoCommands.enableTurretTracking(),
//                        autoCommands.engageClutch(),
//                        autoCommands.spinUpShooter(),
                        driveToShootingPose
                    ),

                    // SHOOT 3 PRELOADS
                    autoCommands.runIntake(),
                    autoCommands.waitForSeconds(3),
                    autoCommands.disengageClutch(),

                    // COLLECT AND SHOOT FIRST LINE
                        firstLineShots
//                    autoCommands.waitForSeconds(0.5),
////                    autoCommands.spinUpShooter(),
//                    autoCommands.engageClutch(),
//                    autoCommands.waitForSeconds(3),
//                    autoCommands.disengageClutch(),
//
//                    // COLLECT AND SHOOT SECOND LINE
                    ,humanPlayerShots
//                    autoCommands.waitForSeconds(0.5),
//                    autoCommands.spinUpShooter(),
//                    autoCommands.engageClutch(),
//                    autoCommands.waitForSeconds(3),
//
//                    // POWER DOWN SUBSYSTEMS
//                    autoCommands.stopIntake(),
//                    autoCommands.disengageClutch(),
//                    autoCommands.stopShooter()
                )
            )
        );
    }
}