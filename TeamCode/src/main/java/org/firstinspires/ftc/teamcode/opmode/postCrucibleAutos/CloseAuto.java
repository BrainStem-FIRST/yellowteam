package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;

import java.util.ArrayList;

@Config
public abstract class CloseAuto extends LinearOpMode {
    // want ability to decide order of collection
    // if i get 12 ball auto:
    //    1: if partner gets 0, 3, or 6 then always collect 2nd one first then open gate
    //    2: if partner gets 6 + gate then order doesn't matter
    //    3: if partner gets 9 with no gate then tell then to only run 6 and do same as scenario 1
    //    4: if partner gets 9 + gate then collect 1st one first and don't open gate
    // if i get 15 ball auto:
    //    1: if partner gets 6 or more than do procedure with 12 ball
    //    2: if partner gets 0 or 3 then collect 3rd one first, then collect 2nd and open gate

    public static class Collect {
        public double maxVel = 20;
        public double lineARed = Math.toRadians(90), lineABlue = 0;

        public double thirdXRed = 35, preThirdYRed = 35, thirdXBlue = 0, preThirdYBlue = 0;
        public double postThirdYRed = 55, postThirdYBlue = 0;

        public double secondStartTRed = Math.toRadians(180), secondStartTBlue = 0;
        public double secondXRed = 12, preSecondYRed = 34, secondXBlue = 0, preSecondYBlue = 0;
        public double postSecondYRed = 52, postSecondYBlue = 0;

        public double firstXRed = -13, preFirstYRed = 47, firstXBlue = 0, preFirstYBlue = 0;
        public double postFirstYRed = 54, postFirstYBlue = 0;

        public double preLoadingXRed = 55, preLoadingYRed = 62, preLoadingARed = Math.toRadians(45), preLoadingXBlue = 0, loadingYBlue = 0, preLoadingABlue = 0;
        public double postLoadingXRed = 68, postLoadingYRed = 60, postLoadingARed = Math.toRadians(25), postLoadingXBlue = 0, postLoadingABlue = 0;

        public double gateCollectStartTRed = Math.toRadians(60), gateCollectStartTBlue = 0;
        public double gateCollectEndTRed = Math.toRadians(90), gateCollectEndTBlue = 0;
        public double gateCollectXRed = 68, gateCollectYRed = 65, gateCollectARed = Math.toRadians(90);
    }
    public static class Misc {
        public double startXRed = -63.5, startYRed = 39.5, startARed = Math.toRadians(0), startXBlue = 0, startYBlue = 0, startABlue = 0;
        public double shootFarXRed = 55, shootFarYRed = 15, shootFarARed = Math.toRadians(135), shootFarXBlue = 0, shootFarYBlue = 0, shootFarABlue = 0;
        public double shootNearXRed = -13, shootNearYRed = 22, shootNearARed = Math.toRadians(110), shootNearXBlue = 0, shootNearYBlue = 0, shootNearABlue = 0;
        public double gateXRed = -5, gateYRed = 56, gateARed = 90, gateTRed = 90, gateXBlue = 0, gateYBlue = 0, gateABlue = 0, gateTBlue = 0;
        public double parkXFarRed = 48, parkYFarRed = 25, parkFarARed = Math.toRadians(135);
        public double parkXNearRed = -12, parkYNearRed = 36, parkANearRed = Math.toRadians(45);
    }
    public static class Customizable {
        public boolean openGateOnFirst = false;
        public boolean openGateOnSecond = true;
        public double minTimeToShootPreload = -1;
        public double minTimeToShootFirstLine = -1;
        public double minTimeToShootSecondLine = -1;
        public double minTimeToShootThirdLine = -1;
        public double minTimeToShootLoadingZone = -1;
        public double minTimeToShootGateCollect = -1;
        public String collectionOrder = "L23G";
    }

    public static class TimeConstraints {
        public double gateWait = 0.2;
        public double minSpinUpWait = 2;
        public double minShootTime = 1.2;
        public double ensureShootAll = 0.4;
        public double extakeDelay = 1;
    }
    public static Customizable customizable = new Customizable();
    public static TimeConstraints timeConstraints = new TimeConstraints();
    public static Collect collect = new Collect();
    public static Misc misc = new Misc();

    protected Alliance alliance;
    private ElapsedTime autoTimer;
    private BrainSTEMRobot robot;
    private AutoCommands autoCommands;

    @Override
    public void runOpMode() throws InterruptedException {
        ArrayList<Double> vels = new ArrayList<>();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);

        autoTimer = new ElapsedTime();
        boolean isRed = alliance == Alliance.RED;

        Pose2d start = isRed ? new Pose2d(misc.startXRed, misc.startYRed, misc.startARed) : new Pose2d(misc.startXBlue, misc.startYBlue, misc.startABlue);
        Pose2d shootFarPose = isRed ? new Pose2d(misc.shootFarXRed, misc.shootFarYRed, misc.shootFarARed) : new Pose2d(misc.shootFarXBlue, misc.shootFarYBlue, misc.shootFarABlue);
        Pose2d shootNearPose = isRed ? new Pose2d(misc.shootNearXRed, misc.shootNearYRed, misc.shootNearARed) : new Pose2d(misc.shootNearXBlue, misc.shootNearYBlue, misc.shootNearABlue);
        Pose2d preCollect1Pose = isRed ? new Pose2d(collect.firstXRed, collect.preFirstYRed, collect.lineARed) : new Pose2d(collect.firstXBlue, collect.preFirstYBlue, collect.lineABlue);
        Pose2d collect1Pose = isRed ? new Pose2d(collect.firstXRed, collect.postFirstYRed, collect.lineARed) : new Pose2d(collect.firstXBlue, collect.postFirstYBlue, collect.lineABlue);
        Pose2d preCollect2Pose = isRed ? new Pose2d(collect.secondXRed, collect.preSecondYRed, collect.lineARed) : new Pose2d(collect.secondXRed, collect.preSecondYBlue, collect.lineABlue);
        Pose2d collect2Pose = isRed ? new Pose2d(collect.secondXRed, collect.postSecondYRed, collect.lineARed) : new Pose2d(collect.secondXBlue, collect.postSecondYBlue, collect.lineABlue);
        Pose2d preCollect3Pose = isRed ? new Pose2d(collect.thirdXRed, collect.preThirdYRed, collect.lineARed) : new Pose2d(collect.thirdXBlue, collect.preThirdYBlue, collect.lineABlue);
        Pose2d collect3Pose = isRed ? new Pose2d(collect.thirdXRed, collect.postThirdYRed, collect.lineARed) : new Pose2d(collect.thirdXBlue, collect.postThirdYBlue, collect.lineABlue);
        Pose2d preLoadingPose = isRed ? new Pose2d(collect.preLoadingXRed, collect.preLoadingYRed, collect.preLoadingARed) : new Pose2d(collect.preLoadingXBlue, collect.loadingYBlue, collect.preLoadingABlue);
        Pose2d postLoadingPose = isRed ? new Pose2d(collect.postLoadingXRed, collect.postLoadingYRed, collect.postLoadingARed) : new Pose2d(collect.postLoadingXBlue, collect.loadingYBlue, collect.postLoadingABlue);
        Pose2d gatePose = isRed ? new Pose2d(misc.gateXRed, misc.gateYRed, misc.gateARed) : new Pose2d(misc.gateXBlue, misc.gateYBlue, misc.gateABlue);
        Pose2d parkNearPose = new Pose2d(misc.parkXNearRed, misc.parkYNearRed, misc.parkANearRed);
        Pose2d gateCollectPose = new Pose2d(collect.gateCollectXRed, collect.gateCollectYRed, collect.gateCollectARed);

        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, start);
        autoCommands = new AutoCommands(robot, telemetry);
        telemetry.addData("start pose", start);
        telemetry.addData("shoot far pos", shootFarPose.position);
        telemetry.addData("shoot far a", Math.toDegrees(shootFarPose.heading.toDouble()));

        Action preloadShootDrive = robot.drive.actionBuilder(start)
                .strafeToLinearHeading(shootFarPose.position, shootFarPose.heading.toDouble())
                .build();

        Action firstCollectDrive = robot.drive.actionBuilder(shootFarPose)
                .splineTo(preCollect1Pose.position, preCollect1Pose.heading.toDouble())
                .splineTo(collect1Pose.position, collect1Pose.heading.toDouble(), new TranslationalVelConstraint(collect.maxVel))
                .build();

        double gateEndT = isRed ? misc.gateTRed : misc.gateTBlue;
        Action firstGateDrive = customizable.openGateOnFirst ?
                new SequentialAction(
                        robot.drive.actionBuilder(collect1Pose).setTangent(0).splineToConstantHeading(gatePose.position, gateEndT).build(),
                        new SleepAction(timeConstraints.gateWait)
                )
                : new SleepAction(0);
        Action firstShootDrive = robot.drive.actionBuilder(customizable.openGateOnFirst ? gatePose : collect1Pose)
                .strafeToLinearHeading(shootNearPose.position, shootNearPose.heading.toDouble())
                .build();


        double startSecondT = isRed ? collect.secondStartTRed : collect.secondStartTBlue;
        Action secondCollectDrive = robot.drive.actionBuilder(shootFarPose)
                .setTangent(startSecondT)
                .splineToLinearHeading(preCollect2Pose, preCollect2Pose.heading.toDouble())
                .splineToLinearHeading(collect2Pose, collect2Pose.heading.toDouble(), new TranslationalVelConstraint(collect.maxVel))
                .build();
        double secondStartGateT = isRed ? Math.toRadians(-135) : 0;
        Action secondGateDrive = customizable.openGateOnSecond ?
                new SequentialAction(
                        robot.drive.actionBuilder(collect2Pose).setTangent(Math.toRadians(secondStartGateT)).splineToConstantHeading(gatePose.position, gateEndT).build(),
                        new SleepAction(timeConstraints.gateWait)
                )
                : new SleepAction(0);
        Action secondShootDrive = robot.drive.actionBuilder(customizable.openGateOnSecond ? gatePose : collect2Pose)
                .strafeToLinearHeading(shootFarPose.position, shootFarPose.heading.toDouble())
                .build();

        Action thirdCollectDrive = robot.drive.actionBuilder(shootFarPose)
                .splineToLinearHeading(preCollect3Pose, preCollect3Pose.heading.toDouble())
                .splineToLinearHeading(collect3Pose, collect3Pose.heading.toDouble(), new TranslationalVelConstraint(collect.maxVel))
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
        double gateCollectStartT = isRed ? collect.gateCollectStartTRed : collect.gateCollectStartTBlue;
        double gateCollectEndT = isRed ? collect.gateCollectEndTRed : collect.gateCollectEndTBlue;
        Action gateCollectDrive = robot.drive.actionBuilder(shootFarPose)
                .setTangent(gateCollectStartT)
                .splineToLinearHeading(gateCollectPose, gateCollectEndT)
                .build();
        Action gateShootDrive = robot.drive.actionBuilder(gateCollectPose)
                .strafeToLinearHeading(shootFarPose.position, shootFarPose.heading.toDouble())
                .build();

        Action parkNearDrive = robot.drive.actionBuilder(customizable.openGateOnFirst ? gatePose : collect1Pose)
                .strafeToLinearHeading(parkNearPose.position, parkNearPose.heading.toDouble())
                .build();

        Action preloadDriveAndShoot = new SequentialAction(
                new ParallelAction(
                        autoCommands.engageClutch(),
                        preloadShootDrive,
                        autoCommands.speedUpShooter(),
                        autoCommands.enableTurretTracking(),
                        new SleepAction(timeConstraints.minSpinUpWait)
                ),
                new SequentialAction(
                        waitUntilMinTime(customizable.minTimeToShootPreload),
                        autoCommands.runIntake(),
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(timeConstraints.minShootTime),
                                        autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll)
                                ),
                                new Action() {
                                    private final ElapsedTime timer = new ElapsedTime();
                                    private boolean first = true;
                                    @Override
                                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                        if(first) {
                                            first = false;
                                            timer.reset();
                                        }
                                        telemetry.addData("shooter vel", robot.shooter.getAvgMotorVelocity());
                                        return timer.seconds() < 3;
                                    }
                                }
                        ),
                        autoCommands.flickerUp(),
                        autoCommands.disengageClutch()
                )
        );

//        Action firstCollectAndShoot = new SequentialAction(
//                firstCollectDrive,
//                autoCommands.stopIntake(),
//                firstGateDrive,
//                autoCommands.runIntake(),
//                firstShootDrive,
//                autoCommands.runIntake(),
//                waitUntilMinTime(customizable.minTimeToShootFirstLine),
//                autoCommands.engageClutch(),
//                new SleepAction(timeConstraints.shootTime),
//                autoCommands.flickerUp(),
//                autoCommands.disengageClutch()
//        );
        Action secondCollectAndShoot = new SequentialAction(
                autoCommands.runIntake(),
                secondCollectDrive,
                autoCommands.stopIntake(),
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(timeConstraints.extakeDelay),
                                autoCommands.engageClutch()
                        ),
                        new SequentialAction(
                                secondGateDrive,
                                secondShootDrive
                        )
                ),
                waitUntilMinTime(customizable.minTimeToShootSecondLine),
                autoCommands.runIntake(),
                new SleepAction(timeConstraints.minShootTime),
                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action thirdCollectAndShoot = new SequentialAction(
                thirdCollectDrive,
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(timeConstraints.extakeDelay),
                                autoCommands.stopIntake(),
                                autoCommands.engageClutch()
                        ),
                        thirdShootDrive
                ),
                waitUntilMinTime(customizable.minTimeToShootThirdLine),
                autoCommands.runIntake(),
                new SleepAction(timeConstraints.minShootTime),
                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action loadingZoneCollectAndShoot = new SequentialAction(
                loadingZoneCollectDrive,
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(timeConstraints.extakeDelay),
                                autoCommands.stopIntake(),
                                autoCommands.engageClutch()
                        ),
                        loadingZoneShootDrive
                ),
                waitUntilMinTime(customizable.minTimeToShootLoadingZone),
                autoCommands.runIntake(),
                new SleepAction(timeConstraints.minShootTime),
                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
        Action gateCollectAndShoot = new SequentialAction(
                gateCollectDrive,
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(timeConstraints.extakeDelay),
                                autoCommands.stopIntake(),
                                autoCommands.engageClutch()
                        ),
                        gateShootDrive
                ),
                waitUntilMinTime(customizable.minTimeToShootGateCollect),
                autoCommands.runIntake(),
                new SleepAction(timeConstraints.minShootTime),
                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );

        int numPaths = customizable.collectionOrder.length();
        ArrayList<Action> actionOrder = new ArrayList<>();
        for(int i = 0; i < numPaths; i++) {
            switch(customizable.collectionOrder.substring(i, i+1).toLowerCase()) {
//                case "1" : actionOrder.add(firstCollectAndShoot); break;
                case "2" : actionOrder.add(secondCollectAndShoot); break;
                case "3" : actionOrder.add(thirdCollectAndShoot); break;
                case "l" : actionOrder.add(loadingZoneCollectAndShoot); break;
                case "g" : actionOrder.add(gateCollectAndShoot); break;
                default: throw new IllegalArgumentException("invalid collectionOrder of " + customizable.collectionOrder + "; can only contain 2, 3, L/l, or G/g");
            }
        }

        Action autoAction = new SequentialAction(
                preloadDriveAndShoot,
                numPaths > 0 ? actionOrder.get(0) : new SleepAction(0),
                numPaths > 1 ? actionOrder.get(1) : new SleepAction(0),
                numPaths > 2 ? actionOrder.get(2) : new SleepAction(0),
                numPaths > 3 ? actionOrder.get(3) : new SleepAction(0)
//                parkNearDrive
        );

        robot.turret.resetEncoders();
        telemetry.addData("alliance", alliance);
//        telemetry.addData("num paths", numPaths);
        telemetry.addLine("READY TO RUN");
        telemetry.update();
        waitForStart();
        autoTimer.reset();

        Actions.runBlocking(
                new ParallelAction(
                        autoAction,
                        autoCommands.updateRobot,
                        telemetryPacket -> {telemetry.update(); return true;}
                )
        );
    }
    private Action waitUntilMinTime(double minTime) {
        return packet -> minTime > 0 && autoTimer.seconds() < minTime;
    }
}
