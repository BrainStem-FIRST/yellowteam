package org.firstinspires.ftc.teamcode.opmode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.AutoPositions;

@Autonomous(name="Red Close Partner Auto", group="Robot")
public class RedClosePartnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime autoTime = new ElapsedTime();
        autoTime.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d startPose = new Pose2d(-63.5, 39.5, Math.toRadians(0));

        BrainSTEMRobot robot = new BrainSTEMRobot(telemetry, hardwareMap, startPose);
        MecanumDrive drive = robot.drive;
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);
        AutoPositions autoPositions = new AutoPositions(drive);

        Action driveToShootingPose = autoPositions.driveCloseShootingPose(startPose);
        Action firstLineShots = autoPositions.firstLineShots(false);
        Action secondLineShots = autoPositions.secondLineShots(false);

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
                    new SleepAction(3),
//                    autoCommands.disengageClutch(),

                    // COLLECT AND SHOOT FIRST LINE
                    firstLineShots,
                    autoCommands.engageClutch(),
                    autoCommands.reverseIntake(),
                    new SleepAction(3),
                    autoCommands.disengageClutch(),
                    autoCommands.runIntake()
//                    autoCommands.waitForSeconds(0.5),
//                    autoCommands.spinUpShooter(),
//                    autoCommands.engageClutch(),
//                    autoCommands.waitForSeconds(3),
//                    autoCommands.disengageClutch(),
//
//                    // COLLECT AND SHOOT HP LINE
                    ,secondLineShots
                    ,autoCommands.engageClutch(),
                    autoCommands.reverseIntake(),
                    new SleepAction(3)
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