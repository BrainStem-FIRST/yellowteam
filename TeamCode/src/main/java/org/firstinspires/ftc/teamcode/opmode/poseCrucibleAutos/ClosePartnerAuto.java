package org.firstinspires.ftc.teamcode.opmode.poseCrucibleAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoPositions;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.TimedAction;

@Config
public abstract class ClosePartnerAuto extends LinearOpMode {
    public static class CustomizableParams {
        public int numSpikeMarksToCollect = 3;
        public boolean collectFromHumanPlayer = false;
        public boolean releaseGateAfterFirstLineCollect = false;
        public boolean releaseGateAfterSecondLineCollect = false;
        public boolean releaseGateAfterThirdLineCollect = false;
    }
    public static class AutoParams {
        public double timeLeftToAbort = 1;
    }
    public static CustomizableParams customParams = new CustomizableParams();
    public static AutoParams params = new AutoParams();
    public final Alliance alliance;
    public ClosePartnerAuto(Alliance alliance) {
        this.alliance = alliance;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime autoTime = new ElapsedTime();
        autoTime.startTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d startPose = alliance == Alliance.RED ? new Pose2d(-63.5, 39.5, 0) : new Pose2d(-63.5, -39.5, 0);

        BrainSTEMRobot robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, startPose);
        robot.turret.resetEncoders();

        MecanumDrive drive = robot.drive;
        AutoCommands autoCommands = new AutoCommands(robot, telemetry);
        AutoPositions autoPositions = new AutoPositions(drive);

        // drive to shoot preloads
        Action driveToShootingPose = alliance == Alliance.RED ? autoPositions.redDriveCloseShootingPose(startPose) : autoPositions.blueDriveCloseShootingPose(startPose);

        // drive to spike marks, collect, then drive to shooting pose
        Action collectAndShootFirstLinePath = alliance == Alliance.RED ?
                autoPositions.redCollectAndShootFirstLine(true, customParams.releaseGateAfterFirstLineCollect) :
                autoPositions.blueCollectAndShootFirstLine(true, customParams.releaseGateAfterFirstLineCollect);

        Action collectAndShootSecondLinePath = alliance == Alliance.RED ?
                autoPositions.redCollectAndShootSecondLine(true, customParams.releaseGateAfterSecondLineCollect) :
                autoPositions.blueCollectAndShootSecondLine(true, customParams.releaseGateAfterSecondLineCollect);

        Action collectAndShootThirdLinePath = alliance == Alliance.RED ?
                autoPositions.redCollectAndShootThirdLine(true, customParams.releaseGateAfterThirdLineCollect) :
                autoPositions.blueCollectAndShootThirdLine(true, customParams.releaseGateAfterThirdLineCollect);

        // move off the line at the end
        Action driveOffLine = alliance == Alliance.RED ?
                autoPositions.redMoveOffLine(true) :
                autoPositions.blueMoveOffLine(true);

        telemetry.addLine("Ready");
        telemetry.setMsTransmissionInterval(20);
        telemetry.update();

        waitForStart();

        Action shootPreloads = new SequentialAction(
                autoCommands.runIntake(),
                new SleepAction(2),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action collectAndShootFirstLine = new SequentialAction(
                collectAndShootFirstLinePath,
                autoCommands.speedUpShooter(),
                autoCommands.engageClutch(),
                new SleepAction(2),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action collectAndShootSecondLine = new SequentialAction(
                collectAndShootSecondLinePath,
                autoCommands.speedUpShooter(),
                autoCommands.engageClutch(),
                new SleepAction(2),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action collectAndShootThirdLine = new SequentialAction(
                collectAndShootThirdLinePath,
                autoCommands.speedUpShooter(),
                autoCommands.engageClutch(),
                new SleepAction(2),
                autoCommands.flickerUp()
        );

        Action autoAction = new ParallelAction(
                autoCommands.updateRobot,
                autoCommands.savePoseContinuously,

                new SequentialAction(
                        new ParallelAction(
                                autoCommands.enableTurretTracking(),
                                autoCommands.engageClutch(),
                                autoCommands.speedUpShooter(),
                                driveToShootingPose
                        ),

                        shootPreloads,
                        decideToAddLine(collectAndShootFirstLine, 1),
                        decideToAddLine(collectAndShootSecondLine, 2),
                        decideToAddLine(collectAndShootThirdLine, 3)
                )
        );

        Action timedAutoAction = new SequentialAction(
                new TimedAction(autoAction, params.timeLeftToAbort),

                autoCommands.turretCenter(),
                driveOffLine
        );

        Actions.runBlocking(timedAutoAction);
    }

    // if our partner collects the far spike mark, we do not want to
    // this function allows us to customize how much of our auto we want to run
    private Action decideToAddLine(Action action, int lineToCollectAndShoot) {
        return lineToCollectAndShoot <= customParams.numSpikeMarksToCollect ? action : packet -> false;
    }
}