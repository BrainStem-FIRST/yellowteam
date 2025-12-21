package org.firstinspires.ftc.teamcode.opmode.preCrucibleAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoPositions;

@Autonomous(name="Crucible RED Auto", group="Robot")
@Config
public class RedClosePartnerAuto extends LinearOpMode {

    private BrainSTEMRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime autoTime = new ElapsedTime();
        autoTime.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d startPose = new Pose2d(-63.5, 39.5, Math.toRadians(0));

        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, startPose);
        robot.turret.resetEncoders();

        MecanumDrive drive = robot.drive;
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);
        AutoPositions autoPositions = new AutoPositions(drive);

        Action driveToShootingPose = autoPositions.redDriveCloseShootingPose(startPose);
        telemetry.addData("close pose", AutoPositions.redCloseShootPose.position.x + " " + AutoPositions.redCloseShootPose.position.y + " " + AutoPositions.redCloseShootPose.heading.toDouble());
        Action firstLineShots = autoPositions.redCollectAndShootFirstLine(true, true, false);
        Action secondLineShots = autoPositions.redCollectAndShootSecondLine(true, true, false);
        Action thirdLineShots = autoPositions.redCollectAndShootThirdLine(true, true, false);
        Action moveOffLine = autoPositions.redMoveOffLine(true);

        telemetry.addLine("Ready");
        telemetry.setMsTransmissionInterval(20);
        telemetry.update();

        waitForStart();

        Actions.runBlocking(

                new ParallelAction(
                        autoCommands.updateRobot,
                        autoCommands.savePoseContinuously,

                        new SequentialAction(
                                new ParallelAction(
                                        autoCommands.enableTurretTracking(),
                                        autoCommands.engageClutch(),
                                        autoCommands.speedUpShooter(),
                                        driveToShootingPose
                                ),

                                // SHOOT 3 PRELOADS
                                autoCommands.runIntake(),
                                new SleepAction(2),
                                autoCommands.flickerUp(),
                                waitForIntakeOn(),
//                                autoCommands.flickerUp(),
                                autoCommands.disengageClutch(),
//                                autoCommands.runIntake(),

                                // COLLECT AND SHOOT FIRST LINE
                                firstLineShots,
                                autoCommands.speedUpShooter(),
                                autoCommands.engageClutch(),
                                new SleepAction(2),
                                autoCommands.flickerUp(),
                                waitForIntakeOn(),
//                                autoCommands.flickerUp(),
                                autoCommands.disengageClutch(),
//                                autoCommands.runIntake(),
                                secondLineShots,

                                autoCommands.speedUpShooter(),
                                autoCommands.engageClutch(),
                                new SleepAction(2),
                                autoCommands.flickerUp(),
                                waitForIntakeOn(),
//                                autoCommands.flickerUp(),
                                autoCommands.disengageClutch(),
//                                autoCommands.runIntake(),
                                thirdLineShots,

                                autoCommands.speedUpShooter(),
                                autoCommands.engageClutch(),
                                new SleepAction(2),
                                autoCommands.flickerUp(),
                                waitForIntakeOn(),
                                autoCommands.disengageClutch(),
                                autoCommands.stopIntake(),
                                autoCommands.turretCenter(),
                                moveOffLine
                        )
                )
        );
    }

    private Action waitForIntakeOn() {
        return telemetryPacket -> robot.collection.getCollectionState() != Collection.CollectionState.INTAKE;
    }
}