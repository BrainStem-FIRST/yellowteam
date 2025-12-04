//package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.opmode.Alliance;
//import org.firstinspires.ftc.teamcode.opmode.testing.PosePredictionErrorRecorder;
//import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
//import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
//import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;
//
//import java.util.ArrayList;
//
//@Config
//public abstract class FarAuto extends LinearOpMode {
//    // want ability to decide order of collection
//    // if i get 12 ball auto:
//    //    1: if partner gets 0, 3, or 6 then always collect 2nd one first then open gate
//    //    2: if partner gets 6 + gate then order doesn't matter
//    //    3: if partner gets 9 with no gate then tell then to only run 6 and do same as scenario 1
//    //    4: if partner gets 9 + gate then collect 1st one first and don't open gate
//    // if i get 15 ball auto:
//    //    1: if partner gets 6 or more than do procedure with 12 ball
//    //    2: if partner gets 0 or 3 then collect 3rd one first, then collect 2nd and open gate
//
//    public static class Collect {
//        public double maxVel = 10;
//        public double lineARed = Math.toRadians(90), lineABlue = 0;
//
//        public double thirdXRed = 35, preThirdYRed = 35, thirdXBlue = 35, preThirdYBlue = -35;
//        public double postThirdYRed = 55, postThirdYBlue = -55;
//
//        public double secondStartTRed = Math.toRadians(180), secondStartTBlue = Math.toRadians(180);
//        public double secondXRed = 12, preSecondYRed = 34, secondXBlue = 12, preSecondYBlue = -34;
//        public double postSecondYRed = 52, postSecondYBlue = -52;
//        public double secondBeginEndVel = 30;
//
//        public double firstXRed = -13, preFirstYRed = 47, firstXBlue = -13, preFirstYBlue = -47;
//        public double postFirstYRed = 54, postFirstYBlue = -54;
//
//        public double preLoadingXRed = 53, preLoadingYRed = 63.5, preLoadingARed = Math.toRadians(45), preLoadingXBlue = 53, loadingYBlue = -63.5, preLoadingABlue = Math.toRadians(-45);
//        public double postLoadingXRed = 68, postLoadingYRed = 60, postLoadingARed = Math.toRadians(25), postLoadingXBlue = 68, postLoadingYBlue = -60, postLoadingABlue = Math.toRadians(-25);
//
//        public double gateCollectStartTRed = Math.toRadians(60), gateCollectStartTBlue = 0;
//        public double gateCollectEndTRed = Math.toRadians(90), gateCollectEndTBlue = 0;
//        public double gateCollectXRed = 68, gateCollectYRed = 65, gateCollectARed = Math.toRadians(90);
//    }
//    public static class Misc {
//        public double startXRed = 63.825, startYRed = 17.6, startARed = Math.toRadians(180), startXBlue = 63.825, startYBlue = -17.6, startABlue = Math.toRadians(180);
//        public double shootFarXRed = 55, shootFarYRed = 15, shootFarXBlue = 55, shootFarYBlue = -15;
//        public double shootFarAFirstRed = Math.toRadians(160), shootFarASecondRed = Math.toRadians(150), shootFarAThirdRed = Math.toRadians(135), shootFarALoadingRed = Math.toRadians(100), shootFarAGateRed = Math.toRadians(85);
//        public double shootFarAFirstBlue = Math.toRadians(-160), shootFarASecondBlue = Math.toRadians(-150), shootFarAThirdBlue = Math.toRadians(-135), shootFarALoadingBlue = Math.toRadians(-100), shootFarAGateBlue = Math.toRadians(-85);
//        public double shootNearXRed = -24, shootNearYRed = 24, shootNearARed = Math.toRadians(110), shootNearXBlue = -24, shootNearYBlue = 24, shootNearABlue = Math.toRadians(-110);
//        public double gateXRed = -5, gateYRed = 56, gateARed = 90, gateTRed = 90, gateXBlue = 0, gateYBlue = 0, gateABlue = 0, gateTBlue = 0;
//        public double parkXFarRed = 48, parkYFarRed = 25, parkFarARed = Math.toRadians(135);
//        public double parkXNearRed = -12, parkYNearRed = 36, parkANearRed = Math.toRadians(45);
//    }
//    public static class Customizable {
//        public boolean openGateOnFirst = false;
//        public boolean openGateOnSecond = true;
//        public double minTimeToShootPreload = -1;
//        public double minTimeToShootFirstLine = -1;
//        public double minTimeToShootSecondLine = -1;
//        public double minTimeToShootThirdLine = -1;
//        public double minTimeToShootLoadingZone = -1;
//        public double minTimeToShootGateCollect = -1;
//        public String collectionOrder = "L23G";
//    }
//
//    public static class TimeConstraints {
//        public double gateWait = 0.2;
//        public double minSpinUpWait = 2;
//        public double minShootTime = 1.2;
//        public double ensureShootAll = 0.4;
//        public double extakeDelay = 1;
//        public double gateCollectMaxTime = 2.1;
//    }
//    public static Customizable customizable = new Customizable();
//    public static TimeConstraints timeConstraints = new TimeConstraints();
//    public static Collect collect = new Collect();
//    public static Misc misc = new Misc();
//
//    protected Alliance alliance;
//    private ElapsedTime autoTimer;
//    private BrainSTEMRobot robot;
//    private AutoCommands autoCommands;
//    private boolean isRed;
//    private Pose2d start, preCollect1Pose, collect1Pose, preCollect2Pose, collect2Pose, preCollect3Pose, collect3Pose, preLoadingPose, postLoadingPose, gatePose, parkNearPose, gateCollectPose;
//    private Pose2d shootFarFirstPose, shootFarSecondPose, shootFarThirdPose, shootFarLoadingPose, shootFarGatePose, shootNearFirstPose;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        ArrayList<Double> vels = new ArrayList<>();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry.setMsTransmissionInterval(11);
//
//        autoTimer = new ElapsedTime();
//        isRed = alliance == Alliance.RED;
//
//        start = isRed ? new Pose2d(misc.startXRed, misc.startYRed, misc.startARed) : new Pose2d(misc.startXBlue, misc.startYBlue, misc.startABlue);
//
//        shootFarFirstPose = isRed ? new Pose2d(misc.shootFarXRed, misc.shootFarYRed, misc.shootFarAFirstRed) : new Pose2d(misc.shootFarXBlue, misc.shootFarYBlue, misc.shootFarAFirstBlue);
//        shootFarSecondPose = isRed ? new Pose2d(misc.shootFarXRed, misc.shootFarYRed, misc.shootFarASecondRed) : new Pose2d(misc.shootFarXBlue, misc.shootFarYBlue, misc.shootFarASecondBlue);
//        shootFarThirdPose = isRed ? new Pose2d(misc.shootFarXRed, misc.shootFarYRed, misc.shootFarAThirdRed) : new Pose2d(misc.shootFarXBlue, misc.shootFarYBlue, misc.shootFarAThirdBlue);
//        shootFarLoadingPose = isRed ? new Pose2d(misc.shootFarXRed, misc.shootFarYRed, misc.shootFarALoadingRed) : new Pose2d(misc.shootFarXBlue, misc.shootFarYBlue, misc.shootFarALoadingBlue);
//        shootFarGatePose = isRed ? new Pose2d(misc.shootFarXRed, misc.shootFarYRed, misc.shootFarAGateRed) : new Pose2d(misc.shootFarXBlue, misc.shootFarYBlue, misc.shootFarAGateBlue);
//        shootNearFirstPose = isRed ? new Pose2d(misc.shootNearXRed, misc.shootNearYRed, misc.shootNearARed) : new Pose2d(misc.shootNearXBlue, misc.shootNearYBlue, misc.shootNearABlue);
//
//        preCollect1Pose = isRed ? new Pose2d(collect.firstXRed, collect.preFirstYRed, collect.lineARed) : new Pose2d(collect.firstXBlue, collect.preFirstYBlue, collect.lineABlue);
//        collect1Pose = isRed ? new Pose2d(collect.firstXRed, collect.postFirstYRed, collect.lineARed) : new Pose2d(collect.firstXBlue, collect.postFirstYBlue, collect.lineABlue);
//        preCollect2Pose = isRed ? new Pose2d(collect.secondXRed, collect.preSecondYRed, collect.lineARed) : new Pose2d(collect.secondXRed, collect.preSecondYBlue, collect.lineABlue);
//        collect2Pose = isRed ? new Pose2d(collect.secondXRed, collect.postSecondYRed, collect.lineARed) : new Pose2d(collect.secondXBlue, collect.postSecondYBlue, collect.lineABlue);
//        preCollect3Pose = isRed ? new Pose2d(collect.thirdXRed, collect.preThirdYRed, collect.lineARed) : new Pose2d(collect.thirdXBlue, collect.preThirdYBlue, collect.lineABlue);
//        collect3Pose = isRed ? new Pose2d(collect.thirdXRed, collect.postThirdYRed, collect.lineARed) : new Pose2d(collect.thirdXBlue, collect.postThirdYBlue, collect.lineABlue);
//        preLoadingPose = isRed ? new Pose2d(collect.preLoadingXRed, collect.preLoadingYRed, collect.preLoadingARed) : new Pose2d(collect.preLoadingXBlue, collect.loadingYBlue, collect.preLoadingABlue);
//        postLoadingPose = isRed ? new Pose2d(collect.postLoadingXRed, collect.postLoadingYRed, collect.postLoadingARed) : new Pose2d(collect.postLoadingXBlue, collect.postLoadingYBlue, collect.postLoadingABlue);
//        gatePose = isRed ? new Pose2d(misc.gateXRed, misc.gateYRed, misc.gateARed) : new Pose2d(misc.gateXBlue, misc.gateYBlue, misc.gateABlue);
//        parkNearPose = new Pose2d(misc.parkXNearRed, misc.parkYNearRed, misc.parkANearRed);
//        gateCollectPose = new Pose2d(collect.gateCollectXRed, collect.gateCollectYRed, collect.gateCollectARed);
//
//        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, start);
//        autoCommands = new AutoCommands(robot, telemetry);
//
//        Action parkNearDrive = robot.drive.actionBuilder(customizable.openGateOnFirst ? gatePose : collect1Pose)
//                .strafeToLinearHeading(parkNearPose.position, parkNearPose.heading.toDouble())
//                .build();
//
//        int numPaths = customizable.collectionOrder.length();
//        if(numPaths == 0)
//            throw new IllegalArgumentException("cannot have empty collectionOrder string");
//        ArrayList<Action> actionOrder = new ArrayList<>();
//        customizable.collectionOrder = customizable.collectionOrder.toLowerCase();
//
//        Pose2d shootPose = shootFarSecondPose;
//        for(int i = 0; i < numPaths; i++) {
//            if(i < numPaths - 1) {
//                String nextLetter = customizable.collectionOrder.substring(i+1, i+2); // g
//                shootPose = getSetupPose(nextLetter);
//            }
//
//            String curLetter = customizable.collectionOrder.substring(i, i+1);
//            switch(curLetter) {
//                case "1" : actionOrder.add(getFirstCollectAndShoot()); break;
//                case "2" : actionOrder.add(getSecondCollectAndShoot(shootPose)); break;
//                case "3" : actionOrder.add(getThirdCollectAndShoot(shootPose)); break;
//                case "l" : actionOrder.add(getLoadingCollectAndShoot(shootPose)); break;
//                case "g" : actionOrder.add(getGateCollectAndShoot(shootPose)); break;
//            }
//        }
//
//        Action autoAction = new SequentialAction(
//                getPreloadDriveAndShoot(getSetupPose(customizable.collectionOrder.substring(0,1))),
//                numPaths > 0 ? actionOrder.get(0) : new SleepAction(0),
//                numPaths > 1 ? actionOrder.get(1) : new SleepAction(0),
//                numPaths > 2 ? actionOrder.get(2) : new SleepAction(0),
//                numPaths > 3 ? actionOrder.get(3) : new SleepAction(0)
////                parkNearDrive
//        );
//
//        robot.turret.resetEncoders();
//        telemetry.addData("alliance", alliance);
////        telemetry.addData("num paths", numPaths);
//        telemetry.addLine("READY TO RUN");
//        telemetry.update();
//        waitForStart();
//        autoTimer.reset();
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        autoAction,
//                        autoCommands.updateRobot,
//                        telemetryPacket -> {telemetry.update(); return true;}
//                )
//        );
//    }
//    private Pose2d getSetupPose(String letter) {
//        switch (letter) {
//            case "1": return shootNearFirstPose;
//            case "2": return shootFarSecondPose;
//            case "3": return shootFarThirdPose;
//            case "l": return shootFarLoadingPose;
//            case "g": return shootFarGatePose;
//            default: throw new IllegalArgumentException("invalid collectionOrder of " + customizable.collectionOrder + "; can only contain 1, 2, 3, L/l, or G/g");
//        }
//    }
//    private Action getPreloadDriveAndShoot(Pose2d shootPose) {
//        Action preloadShootDrive = robot.drive.actionBuilder(start)
//                .strafeToLinearHeading(shootPose.position, shootPose.heading.toDouble())
//                .build();
//        return new SequentialAction(
//                new ParallelAction(
//                        autoCommands.engageClutch(),
//                        preloadShootDrive,
//                        autoCommands.speedUpShooter(),
//                        autoCommands.enableTurretTracking(),
//                        new SleepAction(timeConstraints.minSpinUpWait)
//                ),
//                new SequentialAction(
//                        waitUntilMinTime(customizable.minTimeToShootPreload),
//                        autoCommands.runIntake(),
//                        new SequentialAction(
//                                new SleepAction(timeConstraints.minShootTime),
//                                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll)
//                        ),
//                        autoCommands.flickerUp(),
//                        autoCommands.disengageClutch()
//                )
//        );
//    }
//    private Action getFirstCollectAndShoot() {
//        Action firstCollectDrive = robot.drive.actionBuilder(shootFarFirstPose)
//                .splineTo(preCollect1Pose.position, preCollect1Pose.heading.toDouble())
//                .splineTo(collect1Pose.position, collect1Pose.heading.toDouble(), new TranslationalVelConstraint(collect.maxVel))
//                .build();
//        double gateEndT = isRed ? misc.gateTRed : misc.gateTBlue;
//        Action firstGateDrive = customizable.openGateOnFirst ?
//                new SequentialAction(
//                        robot.drive.actionBuilder(collect1Pose).setTangent(0).splineToConstantHeading(gatePose.position, gateEndT).build(),
//                        new SleepAction(timeConstraints.gateWait)
//                )
//                : new SleepAction(0);
//        Action firstShootDrive = robot.drive.actionBuilder(customizable.openGateOnFirst ? gatePose : collect1Pose)
//                .strafeToLinearHeading(shootNearFirstPose.position, shootNearFirstPose.heading.toDouble())
//                .build();
//
//        return new SequentialAction(
//                autoCommands.runIntake(),
//                firstCollectDrive,
//                autoCommands.stopIntake(),
//                new ParallelAction(
//                        new SequentialAction(
//                                new SleepAction(timeConstraints.extakeDelay),
//                                autoCommands.engageClutch()
//                        ),
//                        new SequentialAction(
//                                firstGateDrive,
//                                firstShootDrive
//                        )
//                ),
//                waitUntilMinTime(customizable.minTimeToShootFirstLine),
//                autoCommands.engageClutch(),
//                new SleepAction(timeConstraints.minShootTime),
//                autoCommands.flickerUp(),
//                autoCommands.disengageClutch()
//        );
//    }
//    private Action getSecondCollectAndShoot(Pose2d shootPose) {
//        double startSecondT = isRed ? collect.secondStartTRed : collect.secondStartTBlue;
//        Action secondCollectDrive = robot.drive.actionBuilder(shootFarSecondPose, collect.secondBeginEndVel)
//                .setTangent(startSecondT)
//                .splineToLinearHeading(preCollect2Pose, preCollect2Pose.heading.toDouble())
//                .splineToLinearHeading(collect2Pose, collect2Pose.heading.toDouble(), new TranslationalVelConstraint(collect.maxVel))
//                .build();
//
//        double secondStartGateT = isRed ? Math.toRadians(-135) : 0;
//        double gateEndT = isRed ? misc.gateTRed : misc.gateTBlue;
//        Action secondGateDrive = customizable.openGateOnSecond ?
//                new SequentialAction(
//                        robot.drive.actionBuilder(collect2Pose).setTangent(Math.toRadians(secondStartGateT)).splineToConstantHeading(gatePose.position, gateEndT).build(),
//                        new SleepAction(timeConstraints.gateWait)
//                )
//                : new SleepAction(0);
//
//        Action secondShootDrive = robot.drive.actionBuilder(customizable.openGateOnSecond ? gatePose : collect2Pose)
//                .strafeToLinearHeading(shootPose.position, shootPose.heading.toDouble())
//                .build();
//
//        return new SequentialAction(
//                autoCommands.runIntake(),
//                secondCollectDrive,
//                autoCommands.stopIntake(),
//                new ParallelAction(
//                        new SequentialAction(
//                                new SleepAction(timeConstraints.extakeDelay),
//                                autoCommands.engageClutch()
//                        ),
//                        new SequentialAction(
//                                secondGateDrive,
//                                secondShootDrive
//                        )
//                ),
//                waitUntilMinTime(customizable.minTimeToShootSecondLine),
//                autoCommands.runIntake(),
//                new SleepAction(timeConstraints.minShootTime),
//                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll),
//                autoCommands.flickerUp(),
//                autoCommands.disengageClutch()
//        );
//    }
//    private Action getThirdCollectAndShoot(Pose2d shootPose) {
//        Action thirdCollectDrive = robot.drive.actionBuilder(shootFarThirdPose)
//                .splineToLinearHeading(preCollect3Pose, preCollect3Pose.heading.toDouble())
//                .splineToLinearHeading(collect3Pose, collect3Pose.heading.toDouble(), new TranslationalVelConstraint(collect.maxVel))
//                .build();
//        Action thirdShootDrive = robot.drive.actionBuilder(collect3Pose)
//                .strafeToLinearHeading(shootPose.position, shootPose.heading.toDouble())
//                .build();
//
//
//        return new SequentialAction(
//                thirdCollectDrive,
//                new ParallelAction(
//                        new SequentialAction(
//                                new SleepAction(timeConstraints.extakeDelay),
//                                autoCommands.stopIntake(),
//                                autoCommands.engageClutch()
//                        ),
//                        thirdShootDrive
//                ),
//                waitUntilMinTime(customizable.minTimeToShootThirdLine),
//                autoCommands.runIntake(),
//                new SleepAction(timeConstraints.minShootTime),
//                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll),
//                autoCommands.flickerUp(),
//                autoCommands.disengageClutch()
//        );
//    }
//    private Action getLoadingCollectAndShoot(Pose2d shootPose) {
//
//        Action loadingCollectDrive = robot.drive.actionBuilder(shootFarLoadingPose)
//                    .strafeToLinearHeading(preLoadingPose.position, preLoadingPose.heading.toDouble())
//                    .strafeToLinearHeading(postLoadingPose.position, postLoadingPose.heading.toDouble())
//                    .build();
//
//        Action loadingShootDrive = robot.drive.actionBuilder(postLoadingPose)
//                    .strafeToLinearHeading(shootPose.position, shootPose.heading.toDouble())
//                    .build();
//
//        return new SequentialAction(
//                loadingCollectDrive,
//                new ParallelAction(
//                        new SequentialAction(
//                                new SleepAction(timeConstraints.extakeDelay),
//                                autoCommands.stopIntake(),
//                                autoCommands.engageClutch()
//                        ),
//                        loadingShootDrive
//                ),
//                waitUntilMinTime(customizable.minTimeToShootLoadingZone),
//                autoCommands.runIntake(),
//                new SleepAction(timeConstraints.minShootTime),
//                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll),
//                autoCommands.flickerUp(),
//                autoCommands.disengageClutch()
//        );
//    }
//    private Action getGateCollectAndShoot(Pose2d shootPose) {
//
//        double gateCollectStartT = isRed ? collect.gateCollectStartTRed : collect.gateCollectStartTBlue;
//        double gateCollectEndT = isRed ? collect.gateCollectEndTRed : collect.gateCollectEndTBlue;
//        Action gateCollectDrive = new CustomEndAction(robot.drive.actionBuilder(shootFarGatePose)
//                .setTangent(gateCollectStartT)
//                .splineToLinearHeading(gateCollectPose, gateCollectEndT)
//                .build(),
//                () -> robot.collection.intakeHas3Balls(),timeConstraints.gateCollectMaxTime);
//        Action gateShootDrive = robot.drive.actionBuilder(gateCollectPose)
//                    .strafeToLinearHeading(shootPose.position, shootPose.heading.toDouble())
//                    .build();
//
//        return new SequentialAction(
//                gateCollectDrive,
//                new ParallelAction(
//                        new SequentialAction(
//                                new SleepAction(timeConstraints.extakeDelay),
//                                autoCommands.stopIntake(),
//                                autoCommands.engageClutch()
//                        ),
//                        gateShootDrive
//                ),
//                waitUntilMinTime(customizable.minTimeToShootGateCollect),
//                autoCommands.runIntake(),
//                new SleepAction(timeConstraints.minShootTime),
//                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll),
//                autoCommands.flickerUp(),
//                autoCommands.disengageClutch()
//        );
//    }
//    private Action waitUntilMinTime(double minTime) {
//        return packet -> minTime > 0 && autoTimer.seconds() < minTime;
//    }
//}
