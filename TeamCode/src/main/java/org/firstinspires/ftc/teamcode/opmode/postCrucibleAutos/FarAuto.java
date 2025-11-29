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
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoRobotStatus;

import java.util.ArrayList;

@Config
public abstract class FarAuto extends LinearOpMode {
    // want ability to decide order of collection
    // if i get 12 ball auto:
    //    1: if partner gets 0, 3, or 6 then always collect 2nd one first then open gate
    //    2: if partner gets 6 + gate then order doesn't matter
    //    3: if partner gets 9 with no gate then tell then to only run 6 and do same as scenario 1
    //    4: if partner gets 9 + gate then collect 1st one first and don't open gate
    // if i get 15 ball auto:
    //    1: if partner gets 6 or more than do procedure with 12 ball
    //    2: if partner gets 0 or 3 then collect 3rd one first, then collect 2nd and open gate

    public static class CollectPoses {
        public double lineARed = Math.toRadians(90), lineABlue = 0;

        public double thirdXRed = 36, preThirdYRed = 0, thirdXBlue = 0, preThirdYBlue = 0;
        public double postThirdYRed = 48, postThirdYBlue = 0;

        public double secondXRed = 12, preSecondYRed = 0, secondXBlue = 0, preSecondYBlue = 0;
        public double postSecondYRed = 48, postSecondYBlue = 0;

        public double firstXRed = -12, preFirstYRed = 0, firstXBlue = 0, preFirstYBlue = 0;
        public double postFirstYRed = 48, postFirstYBlue = 0;

        public double preLoadingXRed = 48, loadingYRed = 60, preLoadingARed = Math.toRadians(45), preLoadingXBlue = 0, loadingYBlue = 0, preLoadingABlue = 0;
        public double postLoadingXRed = 60, postLoadingARed = Math.toRadians(45), postLoadingXBlue = 0, postLoadingABlue = 0;
    }
    public static class MiscPoses {
        public double startXRed = 60, startYRed = 48, startARed = 90, startXBlue = 0, startYBlue = 0, startABlue = 0;
        public double shootFarXRed = 55, shootFarYRed = 30, shootFarARed = Math.toRadians(135), shootFarXBlue = 0, shootFarYBlue = 0, shootFarABlue = 0;
        public double shootNearXRed = -50, shootNearYRed = 50, shootNearARed = Math.toRadians(55), shootNearXBlue = 0, shootNearYBlue = 0, shootNearABlue = 0;
        public double gateXRed = 0, gateYRed = 50, gateARed = 90, gateTRed = 90, gateXBlue = 0, gateYBlue = 0, gateABlue = 0, gateTBlue = 0;
    }
    public static class Customizable {
        public boolean openGateOnFirst = false;
        public boolean openGateOnSecond = true;
        public double minTimeToShootPreload = -1;
        public double minTimeToShootFirstLine = -1;
        public double minTimeToShootSecondLine = -1;
        public double minTimeToShootThirdLine = -1;
        public double minTimeToShootLoadingZone = -1;
        public String collectionOrder = "23L1";
    }

    public static class TimeConstraints {
        public double preload = 0;
        public double preCollect1 = 0, collect1 = 0, collect1ToGate = 0, collect1ToShoot = 0;
        public double preCollect2 = 0, collect2 = 0, collect2ToGate = 0, collect2ToShoot = 0;
        public double preCollect3 = 0, collect3 = 0, collect3ToShoot = 0;
        public double loading = 0, loadingToShoot = 0;

        public double abortTime = 29;
        public double shootTime = 2;
    }
    public static Customizable customizable = new Customizable();
    public static TimeConstraints timeConstraints = new TimeConstraints();
    public static CollectPoses collectPoses = new CollectPoses();
    public static MiscPoses miscPoses = new MiscPoses();

    protected Alliance alliance;
    private ElapsedTime autoTimer;
    private BrainSTEMRobot robot;
    private AutoRobotStatus robotStatus;
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

        autoCommands = new AutoCommands(robot, telemetry);

        Pose2d start = isRed ? new Pose2d(miscPoses.startXRed, miscPoses.startYRed, miscPoses.startARed) : new Pose2d(miscPoses.startXBlue, miscPoses.startYBlue, miscPoses.startABlue);
        Pose2d shootFarPose = isRed ? new Pose2d(miscPoses.shootFarXRed, miscPoses.shootFarYRed, miscPoses.shootFarARed) : new Pose2d(miscPoses.shootFarXBlue, miscPoses.shootFarYBlue, miscPoses.shootFarABlue);
        Pose2d shootNearPose = isRed ? new Pose2d(miscPoses.shootNearXRed, miscPoses.shootNearYRed, miscPoses.shootNearARed) : new Pose2d(miscPoses.shootNearXBlue, miscPoses.shootNearYBlue, miscPoses.shootNearABlue);
        Pose2d collect1Pose = isRed ? new Pose2d(collectPoses.firstXRed, collectPoses.postFirstYRed, collectPoses.lineARed) : new Pose2d(collectPoses.firstXBlue, collectPoses.postFirstYBlue, collectPoses.lineABlue);
        Pose2d collect2Pose = isRed ? new Pose2d(collectPoses.secondXRed, collectPoses.postSecondYRed, collectPoses.lineARed) : new Pose2d(collectPoses.secondXBlue, collectPoses.postSecondYBlue, collectPoses.lineABlue);
        Pose2d collect3Pose = isRed ? new Pose2d(collectPoses.thirdXRed, collectPoses.postThirdYRed, collectPoses.lineARed) : new Pose2d(collectPoses.thirdXBlue, collectPoses.postThirdYBlue, collectPoses.lineABlue);
        Pose2d preLoadingPose = isRed ? new Pose2d(collectPoses.preLoadingXRed, collectPoses.loadingYRed, collectPoses.preLoadingARed) : new Pose2d(collectPoses.preLoadingXBlue, collectPoses.loadingYBlue, collectPoses.preLoadingABlue);
        Pose2d postLoadingPose = isRed ? new Pose2d(collectPoses.postLoadingXRed, collectPoses.loadingYRed, collectPoses.postLoadingARed) : new Pose2d(collectPoses.postLoadingXBlue, collectPoses.loadingYBlue, collectPoses.postLoadingABlue);
        Pose2d gatePose = isRed ? new Pose2d(miscPoses.gateXRed, miscPoses.gateYRed, miscPoses.gateARed) : new Pose2d(miscPoses.gateXBlue, miscPoses.gateYBlue, miscPoses.gateABlue);
        double gateT = isRed ? miscPoses.gateTRed : miscPoses.gateTBlue;

        Action preloadShootDrive = robot.drive.actionBuilder(start)
                .strafeToLinearHeading(shootFarPose.position, shootFarPose.heading.toDouble())
                .build();

        Action firstCollectDrive = robot.drive.actionBuilder(shootFarPose)
                .splineTo(collect1Pose.position, collect1Pose.heading.toDouble())
                .build();
        Action firstGateDrive = customizable.openGateOnFirst ?
                robot.drive.actionBuilder(collect1Pose).splineToConstantHeading(gatePose.position, gateT).build()
                : new SleepAction(0);
        Action firstShootDrive = robot.drive.actionBuilder(customizable.openGateOnFirst ? gatePose : collect1Pose)
                .strafeToLinearHeading(shootNearPose.position, shootNearPose.heading.toDouble())
                .build();


        Action secondCollectDrive = robot.drive.actionBuilder(shootFarPose)
                .splineTo(collect2Pose.position, collect2Pose.heading.toDouble())
                .build();
        Action secondGateDrive = customizable.openGateOnSecond ?
                robot.drive.actionBuilder(collect2Pose).splineToConstantHeading(gatePose.position, gateT).build()
                : new SleepAction(0);
        Action secondShootDrive = robot.drive.actionBuilder(customizable.openGateOnSecond ? gatePose : collect2Pose)
                .strafeToLinearHeading(shootFarPose.position, shootFarPose.heading.toDouble())
                .build();

        Action thirdCollectDrive = robot.drive.actionBuilder(shootFarPose)
                .splineTo(collect3Pose.position, collect3Pose.heading.toDouble())
                .build();
        Action thirdShootDrive = robot.drive.actionBuilder(collect3Pose)
                .strafeToLinearHeading(shootFarPose.position, shootFarPose.heading.toDouble())
                .build();

        Action loadingZoneCollectDrive = robot.drive.actionBuilder(shootFarPose)
                .strafeToLinearHeading(preLoadingPose.position, preLoadingPose.heading.toDouble())
                .strafeToLinearHeading(postLoadingPose.position, postLoadingPose.heading.toDouble())
                .build();
        Action loadingZoneShootDrive = robot.drive.actionBuilder(postLoadingPose)
                .strafeToLinearHeading(shootFarPose.position, shootFarPose.heading.toDouble())
                .build();

        Action preloadDriveAndShoot = new SequentialAction(
                new ParallelAction(
                        autoCommands.disengageClutch(),
                        preloadShootDrive
                ),
                new SequentialAction(
                        autoCommands.runIntake(),
                        waitUntilMinTime(customizable.minTimeToShootPreload),
                        autoCommands.engageClutch(),
                        new SleepAction(timeConstraints.shootTime),
                        autoCommands.flickerUp(),
                        autoCommands.disengageClutch()
                )
        );

        Action firstCollectAndShoot = new SequentialAction(
                firstCollectDrive,
                autoCommands.stopIntake(),
                firstGateDrive,
                firstShootDrive,
                autoCommands.runIntake(),
                waitUntilMinTime(customizable.minTimeToShootFirstLine),
                autoCommands.engageClutch(),
                new SleepAction(timeConstraints.shootTime),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action secondCollectAndShoot = new SequentialAction(
                secondCollectDrive,
                autoCommands.stopIntake(),
                secondGateDrive,
                secondShootDrive,
                autoCommands.runIntake(),
                waitUntilMinTime(customizable.minTimeToShootSecondLine),
                autoCommands.engageClutch(),
                new SleepAction(timeConstraints.shootTime),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action thirdCollectAndShoot = new SequentialAction(
                thirdCollectDrive,
                autoCommands.stopIntake(),
                thirdShootDrive,
                autoCommands.runIntake(),
                waitUntilMinTime(customizable.minTimeToShootThirdLine),
                autoCommands.engageClutch(),
                new SleepAction(timeConstraints.shootTime),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action loadingZoneCollectAndShoot = new SequentialAction(
                loadingZoneCollectDrive,
                autoCommands.stopIntake(),
                loadingZoneShootDrive,
                autoCommands.runIntake(),
                waitUntilMinTime(customizable.minTimeToShootLoadingZone),
                autoCommands.engageClutch(),
                new SleepAction(timeConstraints.shootTime),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );

        int numPaths = customizable.collectionOrder.length();
        ArrayList<Action> actionOrder = new ArrayList<>();
        for(int i = 0; i < numPaths; i++) {
            switch(customizable.collectionOrder.substring(i, i+1).toLowerCase()) {
                case "1" : actionOrder.add(firstCollectAndShoot); break;
                case "2" : actionOrder.add(secondCollectAndShoot); break;
                case "3" : actionOrder.add(thirdCollectAndShoot); break;
                case "l" : actionOrder.add(loadingZoneCollectAndShoot); break;
                default: throw new IllegalArgumentException("invalid collectionOrder of " + customizable.collectionOrder + "; can only contain 1, 2, 3, or L/l");
            }
        }

        Action autoAction = new SequentialAction(
                preloadDriveAndShoot,
                numPaths > 0 ? actionOrder.get(0) : new SleepAction(0),
                numPaths > 1 ? actionOrder.get(1) : new SleepAction(0),
                numPaths > 2 ? actionOrder.get(2) : new SleepAction(0),
                numPaths > 3 ? actionOrder.get(3) : new SleepAction(0)
        );

        telemetry.addData("alliance", alliance);
        telemetry.addData("num paths", numPaths);
        telemetry.addLine("READY TO RUN");
        waitForStart();
        autoTimer.reset();

        Actions.runBlocking(
                new ParallelAction(
                        autoCommands.updateRobot,
                        autoCommands.savePoseContinuously,
                        autoAction,
                        telemetryPacket -> {telemetry.update(); return true;}
                )
        );
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
