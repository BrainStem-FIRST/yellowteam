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
import org.firstinspires.ftc.teamcode.opmode.Alliance;
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

        BrainSTEMRobot robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, startPose);
        robot.turret.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MecanumDrive drive = robot.drive;
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);
        AutoPositions autoPositions = new AutoPositions(drive);

        Action driveToShootingPose = autoPositions.driveCloseShootingPose(startPose);
        Action firstLineShots = autoPositions.firstLineShots(true);
        Action secondLineShots = autoPositions.secondLineShots(true);
        Action thirdLineShots = autoPositions.thirdLineShots(true);
        Action moveOffLine = autoPositions.moveOffLine(true);

        telemetry.addLine("Ready");
        telemetry.setMsTransmissionInterval(20);
        telemetry.update();

        waitForStart();

        Actions.runBlocking(

                new ParallelAction(
                        autoCommands.updateRobot,
                        autoCommands.savePoseContinuously,
//                        autoCommands.saveTurretContinuously,

                        new SequentialAction(
                                new ParallelAction(
                                        autoCommands.enableTurretTracking(),
                                        autoCommands.engageClutch(),
                                        autoCommands.setShooterVelocity(),
                                        driveToShootingPose
                                ),

                                // SHOOT 3 PRELOADS
                                autoCommands.runIntake(),
                                new SleepAction(2),
                                autoCommands.flickerUp(),
//                                autoCommands.flickerUp(),
                                autoCommands.disengageClutch(),
//                                autoCommands.runIntake(),
                                firstLineShots,

                                // COLLECT AND SHOOT FIRST LINE
                                autoCommands.setShooterVelocity(),
                                autoCommands.engageClutch(),
                                new SleepAction(2),
                                autoCommands.flickerUp(),
//                                autoCommands.flickerUp(),
                                autoCommands.disengageClutch(),
//                                autoCommands.runIntake(),
                                secondLineShots,

                                autoCommands.setShooterVelocity(),
                                autoCommands.engageClutch(),
                                new SleepAction(2),
                                autoCommands.flickerUp(),
//                                autoCommands.flickerUp(),
                                autoCommands.disengageClutch(),
//                                autoCommands.runIntake(),
                                thirdLineShots,

                                autoCommands.setShooterVelocity(),
                                autoCommands.engageClutch(),
                                new SleepAction(2),
                                autoCommands.flickerUp(),
                                autoCommands.turretCenter(),
                                moveOffLine
                        )
                )
        );
    }
}