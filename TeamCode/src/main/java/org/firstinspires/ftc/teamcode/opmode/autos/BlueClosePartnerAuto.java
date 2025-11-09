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
import com.qualcomm.robotcore.hardware.DcMotor;
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
        robot.turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MecanumDrive drive = robot.drive;
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);
        AutoPositions autoPositions = new AutoPositions(drive);

        Action blueDriveToShootingPose = autoPositions.blueDriveCloseShootingPose(startPose);
        Action blueFirstLineShots = autoPositions.blueFirstLineShots(true);
        Action blueSecondLineShots = autoPositions.blueSecondLineShots(true);
        Action blueThirdLineShots = autoPositions.blueThirdLineShots(true);
        Action blueMoveOffLine = autoPositions.blueMoveOffLine(true);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(

                new ParallelAction(
                        autoCommands.updateRobot,
                        autoCommands.savePoseContinuously,
//                        autoCommands.saveTurretContinuously,

                        new SequentialAction(
                                autoCommands.setBlueAlliance(),
                                new ParallelAction(
                                        autoCommands.enableTurretTracking(),
                                        autoCommands.engageClutch(),
                                        autoCommands.setShooterVelocity(),
                                        blueDriveToShootingPose
                                ),

                                // SHOOT 3 PRELOADS
                                autoCommands.runIntake(),
                                new SleepAction(1.75),
//                                autoCommands.flickerUp(),
                                autoCommands.disengageClutch(),
//                                autoCommands.runIntake(),
                                blueFirstLineShots,

                                // COLLECT AND SHOOT FIRST LINE
                                autoCommands.setShooterVelocity(),
                                autoCommands.engageClutch(),
                                new SleepAction(2),
//                                autoCommands.flickerUp(),
                                autoCommands.disengageClutch(),
//                                autoCommands.runIntake(),
                                blueSecondLineShots,

                                autoCommands.setShooterVelocity(),
                                autoCommands.engageClutch(),
                                new SleepAction(2.5),
//                                autoCommands.flickerUp(),
                                autoCommands.disengageClutch(),
//                                autoCommands.runIntake(),
                                blueThirdLineShots,

                                autoCommands.setShooterVelocity(),
                                autoCommands.engageClutch(),
                                new SleepAction(2),
                                autoCommands.turretCenter(),
                                blueMoveOffLine
                        )
                )
        );
    }
}