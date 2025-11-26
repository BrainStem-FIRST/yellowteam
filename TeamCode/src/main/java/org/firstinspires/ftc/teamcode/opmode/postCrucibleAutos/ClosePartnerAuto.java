package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

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
        public boolean shootFirstLineClose = true;
        public boolean shootSecondLineClose = true;
        public boolean shootThirdLineClose = true;
        public boolean shootHumanPlayerClose = false;
        // if minTimeToShootFirstLine = 7 & the robot reaches the 1st line shooting position before 7 seconds, it will wait until 7 to shoot
        // if minTimeToShootFirstLine = -1, it will ignore the min time
        public double minTimeToShootPreload = -1;
        public double minTimeToShootFirstLine = -1;
        public double minTimeToShootSecondLine = -1;
        public double minTimeToShootThirdLine = -1;
        public double minTimeToShootHumanPlayer = -1;
        public boolean releaseGateAfterFirstLineCollect = false;
        public boolean releaseGateAfterSecondLineCollect = false;
        public boolean releaseGateAfterThirdLineCollect = false;
    }
    public static class AutoParams {
        public double timeLeftToAbort = 1;
    }
    public static CustomizableParams customParams = new CustomizableParams();
    public static AutoParams autoParams = new AutoParams();
    public final Alliance alliance;
    private double startTime;
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
                autoPositions.redCollectAndShootFirstLine(true, customParams.shootFirstLineClose, customParams.releaseGateAfterFirstLineCollect) :
                autoPositions.blueCollectAndShootFirstLine(true, customParams.shootFirstLineClose, customParams.releaseGateAfterFirstLineCollect);

        Action collectAndShootSecondLinePath = alliance == Alliance.RED ?
                autoPositions.redCollectAndShootSecondLine(true, customParams.shootSecondLineClose, customParams.releaseGateAfterSecondLineCollect) :
                autoPositions.blueCollectAndShootSecondLine(true, customParams.shootSecondLineClose, customParams.releaseGateAfterSecondLineCollect);

        Action collectAndShootThirdLinePath = alliance == Alliance.RED ?
                autoPositions.redCollectAndShootThirdLine(true, customParams.shootThirdLineClose, customParams.releaseGateAfterThirdLineCollect) :
                autoPositions.blueCollectAndShootThirdLine(true, customParams.shootThirdLineClose, customParams.releaseGateAfterThirdLineCollect);

        // drive to human player, collect, then drive to shooting pose
        Action collectAndShootHumanPlayerPath = alliance == Alliance.RED ?
                autoPositions.redCollectAndShootLoadingZone(customParams.shootThirdLineClose, customParams.shootHumanPlayerClose) :
                autoPositions.blueCollectAndShootLoadingZone(customParams.shootThirdLineClose, customParams.shootHumanPlayerClose);

        // move off the line at the end
        Action driveOffLine = alliance == Alliance.RED ?
                autoPositions.redMoveOffLine(true) :
                autoPositions.blueMoveOffLine(true);

        Action shootPreloads = new SequentialAction(
                autoCommands.runIntake(),
                waitUntilMinTime(customParams.minTimeToShootPreload),
                new SleepAction(2),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action collectAndShootFirstLine = new SequentialAction(
                collectAndShootFirstLinePath,
                autoCommands.speedUpShooter(),
                autoCommands.engageClutch(),
                waitUntilMinTime(customParams.minTimeToShootFirstLine),
                new SleepAction(2),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action collectAndShootSecondLine = new SequentialAction(
                collectAndShootSecondLinePath,
                autoCommands.speedUpShooter(),
                autoCommands.engageClutch(),
                waitUntilMinTime(customParams.minTimeToShootSecondLine),
                new SleepAction(2),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action collectAndShootThirdLine = new SequentialAction(
                collectAndShootThirdLinePath,
                autoCommands.speedUpShooter(),
                autoCommands.engageClutch(),
                waitUntilMinTime(customParams.minTimeToShootThirdLine),
                new SleepAction(2),
                autoCommands.flickerUp()
        );
        Action collectAndShootHumanPlayer = new SequentialAction(
                collectAndShootHumanPlayerPath,
                autoCommands.speedUpShooter(),
                autoCommands.engageClutch(),
                waitUntilMinTime(customParams.minTimeToShootHumanPlayer),
                new SleepAction(2),
                autoCommands.flickerUp()
        );

        Action autoAction = new ParallelAction(
                autoCommands.updateRobot,
                autoCommands.savePoseContinuously,
                autoCommands.saveTurretContinuously,

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
                        decideToAddLine(collectAndShootThirdLine, 3),
                        customParams.collectFromHumanPlayer ? collectAndShootHumanPlayer : packet -> false
                )
        );

        Action timedAutoAction = new SequentialAction(
                new TimedAction(autoAction, autoParams.timeLeftToAbort),

                autoCommands.turretCenter(),
                driveOffLine
        );

        telemetry.setMsTransmissionInterval(20);
        addCustomizableAutoTelemetry();
        telemetry.update();

        waitForStart();
        startTime = System.currentTimeMillis() * 0.001;
        Actions.runBlocking(timedAutoAction);
    }

    // if our partner collects the far spike mark, we do not want to
    // this function allows us to customize how much of our auto we want to run
    private Action decideToAddLine(Action action, int lineToCollectAndShoot) {
        return lineToCollectAndShoot <= customParams.numSpikeMarksToCollect ? action : packet -> false;
    }

    private Action waitUntilMinTime(double minTime) {
        return packet -> minTime > 0 && System.currentTimeMillis() * 0.001 - startTime < minTime;
    }

    private void addCustomizableAutoTelemetry() {
        telemetry.addLine("AUTO SPECS");
        int numBalls = customParams.numSpikeMarksToCollect * 3 + (customParams.collectFromHumanPlayer ? 3 : 0);
        int numGateOpens = getGateBoolAsInt(customParams.releaseGateAfterFirstLineCollect) +
                getGateBoolAsInt(customParams.releaseGateAfterSecondLineCollect) +
                getGateBoolAsInt(customParams.releaseGateAfterThirdLineCollect);

        telemetry.addLine(numBalls + " ball auto");
        telemetry.addLine();
        telemetry.addLine(customParams.numSpikeMarksToCollect + " spike marks | " + (customParams.collectFromHumanPlayer ? "collecting from human player" : ""));
        telemetry.addLine("opening gate " + numGateOpens + " times | " +
                customParams.releaseGateAfterFirstLineCollect + ", " +
                customParams.releaseGateAfterSecondLineCollect + ", " +
                customParams.releaseGateAfterThirdLineCollect);
        telemetry.addLine();
        telemetry.addLine(minTimeString("preload", customParams.minTimeToShootPreload));
        telemetry.addLine(minTimeString("first spike", customParams.minTimeToShootFirstLine));
        telemetry.addLine(minTimeString("second spike", customParams.minTimeToShootSecondLine));
        telemetry.addLine(minTimeString("third spike", customParams.minTimeToShootThirdLine));
        telemetry.addLine(minTimeString("human player", customParams.minTimeToShootPreload));
    }

    private int getGateBoolAsInt(boolean openGate) {
        return openGate ? 1 : 0;
    }
    private String minTimeString(String name, double minTime) {
        return minTime < 0 ? "" : name + " min time: " + minTime;
    }
}