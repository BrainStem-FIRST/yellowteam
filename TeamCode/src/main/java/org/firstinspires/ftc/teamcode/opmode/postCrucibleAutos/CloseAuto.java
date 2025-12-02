package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;

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
        public double postThirdYRed = 55.5, postThirdYBlue = 0;

        public double secondStartTRed = Math.toRadians(180), secondStartTBlue = 0;
        public double secondXRed = 12, preSecondYRed = 34, secondXBlue = 0, preSecondYBlue = 0;
        public double postSecondYRed = 52.5, postSecondYBlue = 0;
        public double secondBeginEndVel = 10;

        public double firstXRed = -13, preFirstYRed = 47, firstXBlue = 0, preFirstYBlue = 0;
        public double postFirstYRed = 57, postFirstYBlue = 0;

        public double preLoadingXRed = 53, preLoadingYRed = 63.5, preLoadingARed = Math.toRadians(45), preLoadingXBlue = 0, loadingYBlue = 0, preLoadingABlue = 0;
        public double postLoadingXRed = 68, postLoadingYRed = 60, postLoadingARed = Math.toRadians(25), postLoadingXBlue = 0, postLoadingABlue = 0;

        public double gateCollectStartTRed = Math.toRadians(60), gateCollectStartTBlue = 0;
        public double gateCollectEndTRed = Math.toRadians(90), gateCollectEndTBlue = 0;
        public double gateCollectXRed = 68, gateCollectYRed = 65, gateCollectARed = Math.toRadians(90);
    }
    public static class Misc {
        public double startXRed = -63.5, startYRed = 39.5, startARed = 0, startXBlue = 0, startYBlue = 0, startABlue = 0;
        public double shootNearXRed = -14, shootNearYRed = 20, shootNearXBlue = 0, shootNearYBlue = 0;
        public double shootFarXRed = 55, shootFarYRed = 15, shootFarXBlue = 0, shootFarYBlue = 0;

        // custom shooting angles
        public double shootSetup1ARed = Math.toRadians(90), shootSetup2ARed = Math.toRadians(60), shootNearSetup3ARed = Math.toRadians(60), shootNearSetupLoadingARed = Math.toRadians(45);
        public double shootFarSetup3ARed = 150, shootFarSetupLoadingARed = 150;
        public double shootSetup1ABlue = Math.toRadians(160), shootSetup2ABlue = Math.toRadians(150), shootNearSetup3ABlue = Math.toRadians(135), shootNearSetupLoadingABlue = Math.toRadians(90);
        public double shootFarSetup3ABlue = -150, shootFarSetupLoadingABlue = -150;

        public double gateXRed = -7, gateYRed = 60,  gateXBlue = 0, gateYBlue = 0;
        public double gateA1 = Math.toRadians(180), gateA2 = Math.toRadians(0);
        public double parkXFarRed = 48, parkYFarRed = 25, parkFarARed = Math.toRadians(135);
        public double parkXNearRed = -12, parkYNearRed = 36, parkANearRed = Math.toRadians(45);

    }
    public static class Customizable {
        public boolean openGateOnFirst = false;
        public boolean openGateOnSecond = true;
        public boolean shootSecondClose = false, shootThirdClose = true;
        public double minTimeToShootPreload = -1;
        public double minTimeToShootFirstLine = -1;
        public double minTimeToShootSecondLine = -1;
        public double minTimeToShootThirdLine = -1;
        public double minTimeToShootLoadingZone = -1;
        public String collectionOrder = "123";
    }

    public static class TimeConstraints {
        public double gateWait = 0;
        public double minShootTime = 1.2;
        public double ensureShootAll = 0.4;
    }
    public static Customizable customizable = new Customizable();
    public static TimeConstraints timeConstraints = new TimeConstraints();
    public static Collect collect = new Collect();
    public static Misc misc = new Misc();

    protected Alliance alliance;
    private ElapsedTime autoTimer;
    private BrainSTEMRobot robot;
    private AutoCommands autoCommands;
    private Pose2d start, collect1Pose, preCollect2Pose, collect2Pose, preCollect3Pose, collect3Pose, preLoadingPose, postLoadingPose, gate1Pose, gate2Pose, parkNearPose, gateCollectPose;
    private Pose2d shootSetup1Pose, shootSetup2Pose, shootNearSetup3Pose, shootFarSetup3Pose, shootNearSetupLoadingPose, shootFarSetupLoadingPose;

    public Pose2d shootNearRed(double angleRad) { return new Pose2d(misc.shootNearXRed, misc.shootNearYRed, angleRad); }
    public Pose2d shootFarRed(double angleRad) { return new Pose2d(misc.shootFarXRed, misc.shootFarYRed, angleRad); }
    public Pose2d shootNearBlue(double angleRad) { return new Pose2d(misc.shootNearXBlue, misc.shootNearYBlue, angleRad); }
    public Pose2d shootFarBlue(double angleRad) { return new Pose2d(misc.shootFarXBlue, misc.shootFarYBlue, angleRad); }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);

        autoTimer = new ElapsedTime();
        boolean isRed = alliance == Alliance.RED;

        start = isRed ? new Pose2d(misc.startXRed, misc.startYRed, misc.startARed) : new Pose2d(misc.startXBlue, misc.startYBlue, misc.startABlue);

        shootSetup1Pose = isRed ? shootNearRed(misc.shootSetup1ARed) : shootNearBlue(misc.shootSetup1ABlue);
        shootSetup2Pose = isRed ? shootNearRed(misc.shootSetup2ARed) : shootNearBlue(misc.shootSetup2ABlue);
        shootNearSetup3Pose = isRed ? shootNearRed(misc.shootNearSetup3ARed) : shootNearBlue(misc.shootNearSetup3ABlue);
        shootFarSetup3Pose = isRed ? shootFarRed(misc.shootFarSetup3ARed) : shootFarBlue(misc.shootFarSetup3ABlue);
        shootNearSetupLoadingPose = isRed ? shootNearRed(misc.shootNearSetupLoadingARed) : shootNearBlue(misc.shootNearSetupLoadingABlue);
        shootFarSetupLoadingPose = isRed ? shootFarRed(misc.shootFarSetupLoadingARed) : shootFarBlue(misc.shootFarSetupLoadingABlue);

        collect1Pose = isRed ?
                new Pose2d(collect.firstXRed, collect.postFirstYRed, collect.lineARed) :
                new Pose2d(collect.firstXBlue, collect.postFirstYBlue, collect.lineABlue);
        preCollect2Pose = isRed ?
                new Pose2d(collect.secondXRed, collect.preSecondYRed, collect.lineARed) :
                new Pose2d(collect.secondXRed, collect.preSecondYBlue, collect.lineABlue);
        collect2Pose = isRed ?
                new Pose2d(collect.secondXRed, collect.postSecondYRed, collect.lineARed) :
                new Pose2d(collect.secondXBlue, collect.postSecondYBlue, collect.lineABlue);
        preCollect3Pose = isRed ?
                new Pose2d(collect.thirdXRed, collect.preThirdYRed, collect.lineARed) :
                new Pose2d(collect.thirdXBlue, collect.preThirdYBlue, collect.lineABlue);
        collect3Pose = isRed ?
                new Pose2d(collect.thirdXRed, collect.postThirdYRed, collect.lineARed) :
                new Pose2d(collect.thirdXBlue, collect.postThirdYBlue, collect.lineABlue);
        preLoadingPose = isRed ?
                new Pose2d(collect.preLoadingXRed, collect.preLoadingYRed, collect.preLoadingARed) :
                new Pose2d(collect.preLoadingXBlue, collect.loadingYBlue, collect.preLoadingABlue);
        postLoadingPose = isRed ?
                new Pose2d(collect.postLoadingXRed, collect.postLoadingYRed, collect.postLoadingARed) :
                new Pose2d(collect.postLoadingXBlue, collect.loadingYBlue, collect.postLoadingABlue);
        gate1Pose = isRed ?
                new Pose2d(misc.gateXRed, misc.gateYRed, misc.gateA1) :
                new Pose2d(misc.gateXBlue, misc.gateYBlue, misc.gateA1);
        gate2Pose = isRed ?
                new Pose2d(misc.gateXRed, misc.gateYRed, misc.gateA2) :
                new Pose2d(misc.gateXBlue, misc.gateYBlue, misc.gateA2);

        parkNearPose = new Pose2d(misc.parkXNearRed, misc.parkYNearRed, misc.parkANearRed);
        gateCollectPose = new Pose2d(collect.gateCollectXRed, collect.gateCollectYRed, collect.gateCollectARed);

        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, start);
        autoCommands = new AutoCommands(robot, telemetry);

        int numPaths = customizable.collectionOrder.length();
        if(numPaths == 0)
            throw new IllegalArgumentException("cannot have empty collectionOrder string");
        ArrayList<Action> actionOrder = new ArrayList<>();
        customizable.collectionOrder = customizable.collectionOrder.toLowerCase();

        Pose2d shootPose = shootSetup2Pose;
        for(int i = 0; i < numPaths; i++) {
            if(i < numPaths - 1) {
                String nextLetter = customizable.collectionOrder.substring(i+1, i+2); // g
                shootPose = getSetupPose(nextLetter);
            }

            String curLetter = customizable.collectionOrder.substring(i, i+1);
            switch(curLetter) {
                case "1" : actionOrder.add(getFirstCollectAndShoot(shootPose)); break;
                case "2" : actionOrder.add(getSecondCollectAndShoot(shootPose)); break;
                case "3" : actionOrder.add(getThirdCollectAndShoot(shootPose)); break;
                case "l" : actionOrder.add(getLoadingCollectAndShoot(shootPose)); break;
            }
        }

        Action autoAction = new SequentialAction(
                getPreloadDriveAndShoot(getSetupPose(customizable.collectionOrder.substring(0,1))),
                actionOrder.get(0),
                numPaths > 1 ? actionOrder.get(1) : new SleepAction(0),
                numPaths > 2 ? actionOrder.get(2) : new SleepAction(0),
                numPaths > 3 ? actionOrder.get(3) : new SleepAction(0)
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
    private Pose2d getSetupPose(String letter) {
        switch (letter) {
            case "1": return shootSetup1Pose;
            case "2": return shootSetup2Pose;
            case "3": return customizable.shootSecondClose ? shootNearSetup3Pose : shootFarSetup3Pose;
            case "l": return customizable.shootThirdClose ? shootNearSetupLoadingPose : shootFarSetupLoadingPose;
            default: throw new IllegalArgumentException("invalid collectionOrder of " + customizable.collectionOrder + "; can only contain 1, 2, 3, L/l, or G/g");
        }
    }

    private Action getPreloadDriveAndShoot(Pose2d shootPose) {
        Action preloadShootDrive = robot.drive.actionBuilder(start)
                .strafeToLinearHeading(shootPose.position, shootPose.heading.toDouble())
                .build();
        return new SequentialAction(
                new ParallelAction(
                        autoCommands.engageClutch(),
                        preloadShootDrive,
                        autoCommands.speedUpShooter(),
                        autoCommands.enableTurretTracking()
                ),
                new SequentialAction(
                        waitUntilMinTime(customizable.minTimeToShootPreload),
                        autoCommands.runIntake(),
                        new SequentialAction(
                                new SleepAction(timeConstraints.minShootTime),
                                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll)
                        ),
                        autoCommands.flickerUp(),
                        autoCommands.disengageClutch()
                )
        );
    }
    private Action getFirstCollectAndShoot(Pose2d shootPose) {
        Action firstCollectDrive = new CustomEndAction(
                robot.drive.actionBuilder(shootSetup1Pose)
                .strafeToLinearHeading(collect1Pose.position, collect1Pose.heading.toDouble())
                .build(),
        () -> robot.drive.localizer.getPose().position.y > collect1Pose.position.y - 0.5 || robot.collection.intakeHas3Balls(),
                10
        );
        Action firstGateDrive = customizable.openGateOnFirst ?
                new SequentialAction(
                        robot.drive.actionBuilder(collect1Pose).strafeToLinearHeading(gate1Pose.position, gate1Pose.heading.toDouble()).build(),
                        new SleepAction(timeConstraints.gateWait)
                )
                : new SleepAction(0);
        Action firstShootDrive = robot.drive.actionBuilder(customizable.openGateOnFirst ? gate1Pose : collect1Pose)
                .strafeToLinearHeading(shootPose.position, shootPose.heading.toDouble())
                .build();

        return new SequentialAction(
                autoCommands.runIntake(),
                firstCollectDrive,
                autoCommands.engageClutch(),
                autoCommands.stopIntake(),
                firstGateDrive,
                firstShootDrive,
                waitUntilMinTime(customizable.minTimeToShootFirstLine),
                autoCommands.runIntake(),
                new SleepAction(timeConstraints.minShootTime),
                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
    }
    private Action getSecondCollectAndShoot(Pose2d shootPose) {
        Action secondCollectDrive = new CustomEndAction(
                robot.drive.actionBuilder(shootSetup2Pose, collect.secondBeginEndVel)
                .setTangent(0)
                .splineToSplineHeading(preCollect2Pose, preCollect2Pose.heading.toDouble())
                .splineToLinearHeading(collect2Pose, collect2Pose.heading.toDouble(), new TranslationalVelConstraint(collect.maxVel))
                .build(),
                () -> robot.drive.localizer.getPose().position.y > collect2Pose.position.y - 0.5 || robot.collection.intakeHas3Balls(),
                10
        );

        Action secondGateDrive = customizable.openGateOnSecond ?
                new SequentialAction(
                        robot.drive.actionBuilder(collect2Pose).strafeToLinearHeading(gate2Pose.position, gate2Pose.heading.toDouble()).build(),
                        new SleepAction(timeConstraints.gateWait)
                )
                : new SleepAction(0);

        Action secondShootDrive = robot.drive.actionBuilder(customizable.openGateOnSecond ? gate2Pose : collect2Pose)
                .strafeToLinearHeading(shootPose.position, shootPose.heading.toDouble())
                .build();

        return new SequentialAction(
                autoCommands.runIntake(),
                secondCollectDrive,
                autoCommands.engageClutch(),
                autoCommands.stopIntake(),
                secondGateDrive,
                secondShootDrive,
                waitUntilMinTime(customizable.minTimeToShootSecondLine),
                autoCommands.runIntake(),
                new SleepAction(timeConstraints.minShootTime),
                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
    }
    private Action getThirdCollectAndShoot(Pose2d shootPose) {
        Action thirdCollectDrive = new CustomEndAction(
                robot.drive.actionBuilder(shootNearSetup3Pose)
                .setTangent(0)
                .splineToSplineHeading(preCollect3Pose, preCollect3Pose.heading.toDouble())
                .splineToLinearHeading(collect3Pose, collect3Pose.heading.toDouble(), new TranslationalVelConstraint(collect.maxVel))
                .build(),
                () -> robot.drive.localizer.getPose().position.y > collect3Pose.position.y - 0.5 || robot.collection.intakeHas3Balls(),
                10
        );
        Action thirdShootDrive = robot.drive.actionBuilder(collect3Pose)
                .strafeToLinearHeading(shootPose.position, shootPose.heading.toDouble())
                .build();

        return new SequentialAction(
                autoCommands.runIntake(),
                thirdCollectDrive,
                autoCommands.engageClutch(),
                autoCommands.stopIntake(),
                thirdShootDrive,
                waitUntilMinTime(customizable.minTimeToShootThirdLine),
                autoCommands.runIntake(),
                new SleepAction(timeConstraints.minShootTime),
                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
    }
    private Action getLoadingCollectAndShoot(Pose2d shootPose) {

        Action loadingCollectDrive = robot.drive.actionBuilder(shootFarSetupLoadingPose)
                .strafeToLinearHeading(preLoadingPose.position, preLoadingPose.heading.toDouble())
                .strafeToLinearHeading(postLoadingPose.position, postLoadingPose.heading.toDouble())
                .build();

        Action loadingShootDrive = robot.drive.actionBuilder(postLoadingPose)
                .strafeToLinearHeading(shootPose.position, shootPose.heading.toDouble())
                .build();

        return new SequentialAction(
                loadingCollectDrive,
                autoCommands.engageClutch(),
                autoCommands.stopIntake(),
                loadingShootDrive,
                waitUntilMinTime(customizable.minTimeToShootLoadingZone),
                autoCommands.runIntake(),
                new SleepAction(timeConstraints.minShootTime),
                autoCommands.waitTillDoneShooting(timeConstraints.ensureShootAll),
                autoCommands.flickerUp(),
                autoCommands.disengageClutch()
        );
    }
    private Action waitUntilMinTime(double minTime) {
        return packet -> minTime > 0 && autoTimer.seconds() < minTime;
    }
}
