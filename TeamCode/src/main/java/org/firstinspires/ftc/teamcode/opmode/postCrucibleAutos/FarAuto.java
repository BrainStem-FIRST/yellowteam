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
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoPositions;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoRobotStatus;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.TimedAction;

@Config
public abstract class FarAuto extends LinearOpMode {
    public static class CustomizableParams {
        public boolean openGateOnSecond = true;
        public boolean openGateOnThird = false;
        public double minTimeToShootPreload = -1;
        public double minTimeToShootFirstLine = -1;
        public double minTimeToShootSecondLine = -1;
        public double minTimeToShootThirdLine = -1;
        public double minTimeToShootHumanPlayer = -1;
    }
    public static CustomizableParams customizableParams = new CustomizableParams();
    public static class AutoParams {
        public double abortTime = 29;
        public double shootTime = 2;
    }
    public static AutoParams autoParams = new AutoParams();
    private final Alliance alliance;
    public FarAuto(Alliance alliance) {
        this.alliance = alliance;
    }
    private ElapsedTime autoTimer;
    private BrainSTEMRobot robot;
    private AutoRobotStatus robotStatus;
    private AutoPositions autoPositions;
    private AutoCommands autoCommands;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);

        robotStatus = new AutoRobotStatus();

        autoTimer = new ElapsedTime();
        boolean isRed = alliance == Alliance.RED;
        Pose2d startPose = isRed ? new Pose2d(0, 0, 0) : new Pose2d(1, 0, 0);
        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, startPose);

        autoPositions = new AutoPositions(robot.drive);
        autoCommands = new AutoCommands(robot, telemetry);

        // drive to shoot preloads
        Action driveToShootingPose = isRed ? autoPositions.redDriveFarShootingPose(startPose) : autoPositions.blueDriveFarShootingPose(startPose);
        Action driveAndShootPreloads = new SequentialAction(
                new ParallelAction(
                        autoCommands.disengageClutch(),
                        autoCommands.enableTurretTracking(),
                        autoCommands.speedUpShooter(),
                        driveToShootingPose
                ),
                new SequentialAction(
                        autoCommands.runIntake(),
                        waitUntilMinTime(customizableParams.minTimeToShootPreload),
                        autoCommands.engageClutch(),
                        new SleepAction(autoParams.shootTime),
                        autoCommands.flickerUp(),
                        autoCommands.disengageClutch()
                )
        );

        Action driveToCollectAndShootFirst = autoPositions.collectAndShoot(Alliance.RED, 1,
                false,
                isRed ? AutoPositions.redFarShootingPosition : autoPositions.blueFarShootingPosition,
                0,
                isRed ? AutoPositions.redFirstLine : autoPositions.blueFirstLine,
                0, 0, 0, 0,
                robotStatus);
        Action driveToCollectAndShootSecond = autoPositions.collectAndShoot(Alliance.RED, 2,
                customizableParams.openGateOnSecond,
                isRed ? AutoPositions.redFarShootingPosition : autoPositions.blueFarShootingPosition,
                0,
                isRed ? AutoPositions.redSecondLine : autoPositions.blueSecondLine,
                0, 0, 0, 0,
                robotStatus);
        Action driveToCollectAndShootLoadingZone = isRed ? autoPositions.redCollectAndShootLoadingZone(false, false)
                : autoPositions.blueCollectAndShootLoadingZone(false, false);
        Action driveToCollectAndShootThird = autoPositions.collectAndShoot(Alliance.RED,3,
                customizableParams.openGateOnThird,
                isRed ? AutoPositions.redFarShootingPosition : autoPositions.blueFarShootingPosition,
                0,
                isRed ? AutoPositions.redThirdLine : autoPositions.blueThirdLine,
                0, 0, 0, 0,
                robotStatus);

        Action firstCollectAndShoot = new SequentialAction(
                new ParallelAction(
                        driveToCollectAndShootFirst,
                        updateIntakeDuringDrive()
                ),
                autoCommands.runIntake(),
                waitUntilMinTime(customizableParams.minTimeToShootFirstLine),
                autoCommands.engageClutch(),
                new SleepAction(autoParams.shootTime),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action secondCollectAndShoot = new SequentialAction(
                new ParallelAction(
                        driveToCollectAndShootSecond,
                        updateIntakeDuringDrive()
                ),
                autoCommands.runIntake(),
                waitUntilMinTime(customizableParams.minTimeToShootSecondLine),
                autoCommands.engageClutch(),
                new SleepAction(autoParams.shootTime),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action loadingZoneCollectAndShoot = new SequentialAction(
                new ParallelAction(
                        driveToCollectAndShootLoadingZone,
                        updateIntakeDuringDrive()
                ),
                autoCommands.runIntake(),
                waitUntilMinTime(customizableParams.minTimeToShootHumanPlayer),
                autoCommands.engageClutch(),
                new SleepAction(autoParams.shootTime),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action thirdCollectAndShoot = new SequentialAction(
                new ParallelAction(
                        driveToCollectAndShootThird,
                        updateIntakeDuringDrive()
                ),
                autoCommands.runIntake(),
                waitUntilMinTime(customizableParams.minTimeToShootHumanPlayer),
                autoCommands.engageClutch(),
                new SleepAction(autoParams.shootTime),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );


        Action autoAction = new SequentialAction(
                driveAndShootPreloads,
                firstCollectAndShoot,
                secondCollectAndShoot,
                loadingZoneCollectAndShoot,
                thirdCollectAndShoot
        );

        Action exitLineAction = alliance == Alliance.RED ?
                autoPositions.redMoveOffLine(false) :
                autoPositions.blueMoveOffLine(false);

        waitForStart();
        autoTimer.reset();

        Actions.runBlocking(new SequentialAction(
                new TimedAction(autoAction, autoParams.abortTime),
                new ParallelAction(
                        autoCommands.turretCenter(),
                        exitLineAction
                )
        ));
    }
    private Action waitUntilMinTime(double minTime) {
        return packet -> minTime > 0 && autoTimer.seconds() < minTime;
    }
    private Action updateIntakeDuringDrive() {
        return telemetryPacket -> {
            if(robotStatus.postCollecting) {
                autoCommands.stopIntake();
                return false;
            }
            return true;
        };
    }
}
