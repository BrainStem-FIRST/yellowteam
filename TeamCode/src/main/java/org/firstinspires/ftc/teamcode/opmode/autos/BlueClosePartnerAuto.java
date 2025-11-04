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

@Autonomous(name="Blue Close Partner Auto", group="Robot")
public class BlueClosePartnerAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime autoTime = new ElapsedTime();
        autoTime.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d startPose = new Pose2d(-63.5, -39.5, Math.toRadians(0));

        BrainSTEMRobot robot = new BrainSTEMRobot(telemetry, hardwareMap, startPose);
        MecanumDrive drive = robot.drive;
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);
        AutoPositions autoPositions = new AutoPositions(drive);

        Action blueDriveToShootingPose = autoPositions.blueDriveCloseShootingPose(startPose);
        Action blueFirstLineShots = autoPositions.blueFirstLineShots(true);
        Action blueSecondLineShots = autoPositions.blueSecondLineShots(true);

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
                                        autoCommands.enableTurretTracking(),
                                        autoCommands.engageClutch(),
                                        autoCommands.spinUpShooter(true),
                                        blueDriveToShootingPose
                                ),

                                // SHOOT 3 PRELOADS
                                autoCommands.runIntake(),
                                new SleepAction(3.5),
                                new ParallelAction(
                                        blueFirstLineShots,
                                        new SequentialAction(
                                                autoCommands.reverseIntake(),
                                                new SleepAction(2),
                                                autoCommands.runIntake(),
                                                autoCommands.disengageClutch()
                                        )
                                ),

                                // COLLECT AND SHOOT FIRST LINE
                                autoCommands.spinUpShooter(true),
                                autoCommands.engageClutch(),
                                new SleepAction(3.5),

                                new ParallelAction(
                                        blueSecondLineShots,
                                        new SequentialAction(
                                                autoCommands.reverseIntake(),
                                                new SleepAction(2),
                                                autoCommands.runIntake(),
                                                autoCommands.disengageClutch()
                                        )
                                ),
                                autoCommands.spinUpShooter(true),
                                autoCommands.engageClutch(),
                                new SleepAction(2)
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