package org.firstinspires.ftc.teamcode.opmode.postCompAutos;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.CustomEndAction;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.TimedAction;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

import java.util.ArrayList;

@Config
public abstract class AutoPid extends LinearOpMode {
    // want ability to decide order of collection
    // if i get 12 ball auto:
    //    1: if partner gets 0, 3, or 6 then always collect 2nd one first then open gate
    //    2: if partner gets 6 + gate then order doesn't matter
    //    3: if partner gets 9 with no gate then tell then to only run 6 and do same as scenario 1
    //    4: if partner gets 9 + gate then collect 1st one first and don't open gate
    // if i get 15 ball auto:
    //    1: if partner gets 6 or more than do procedure with 12 ball
    //    2: if partner gets 0 or 3 then collect 3rd one first, then collect 2nd and open gate
    public static boolean paused = false;
    public static class Customizable {
        public String collectionOrder = "n2ngngn1n";
        public String minTimes = "-1,-1,-1,-1,-1";
        public boolean openGateOnFirst = false;
        public boolean openGateOnSecond = false;
        public boolean useParkAbort = true;
        public int maxCornerRetries = 0;
    }
    public enum AutoState {
        DRIVE_TO_COLLECT,
        OPEN_GATE,
        DRIVE_TO_SHOOT,
        SHOOT
    }
    public static Customizable customizable = new Customizable();
    public static AutoParamsPid.TimeConstraints timeConstraints = new AutoParamsPid.TimeConstraints();
    public static AutoParamsPid.Collect collect = new AutoParamsPid.Collect();
    public static AutoParamsPid.Shoot shoot = new AutoParamsPid.Shoot();
    public static AutoParamsPid.Misc misc = new AutoParamsPid.Misc();

    protected Alliance alliance;
    private ElapsedTime autoTimer;
    private BrainSTEMRobot robot;
    private AutoCommands autoCommands;
    private Pose2d start,
            collect1Pose, preCollect1Pose,
            preCollect2Pose, collect2Pose,
            preCollect3Pose, collect3Pose,
            preLoadingPose, postLoadingPose,
            cornerCollectPose, cornerCollectRetryPose,
            gateCollectWaypointPose, gateCollectOpenPose, gateCollectPose, gateCollectBackupPose,
            gate1Pose, gate2Pose,
            parkNearPose, parkFarPose,
            shootFar1WaypointPose, shootFar2WaypointPose, shoot2NearWaypointPose,
            shootNearSetup1Pose, shootFarSetup1Pose,
            shootNearSetup2Pose, shootFarSetup2Pose,
            shootNearSetup3Pose, shootFarSetup3Pose,
            shootNearSetupLoadingPose, shootFarSetupLoadingPose;
    private boolean isRed;
    private AutoState autoState;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);

        autoTimer = new ElapsedTime();
        isRed = alliance == Alliance.RED;

        if(customizable.collectionOrder.charAt(0) == 'n')
            start = isRed ? new Pose2d(misc.startNearXRed, misc.startNearYRed, misc.startNearARed) : new Pose2d(misc.startNearXBlue, misc.startNearYBlue, misc.startNearABlue);
        else if(customizable.collectionOrder.charAt(0) == 'f')
            start = isRed ? new Pose2d(misc.startFarXRed, misc.startFarYRed, misc.startFarARed) : new Pose2d(misc.startFarXBlue, misc.startFarYBlue, misc.startFarABlue);

        // DECLARE POSES=======================
        declareShootPoses();
        declareCollectPoses();
        declareMiscPoses();

        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, start);
        autoCommands = new AutoCommands(robot, telemetry);

        int numPaths = (customizable.collectionOrder.length()-1) / 2; //3
        if(numPaths == 0)
            throw new IllegalArgumentException("cannot have empty collectionOrder string");
        ArrayList<Action> actionOrder = new ArrayList<>();
        customizable.collectionOrder = customizable.collectionOrder.toLowerCase();

        String[] minTimeStr = customizable.minTimes.split(",");
        double[] minTimes = new double[minTimeStr.length];
        if(minTimeStr.length != numPaths + 1)
            telemetry.addLine("min time string currently has " + minTimeStr.length + " nums. It must have " + (numPaths+1) + " nums. (1 more than collectionOrder)");
        for (int i=0; i<minTimeStr.length; i++) {
            try {
                minTimes[i] = Double.parseDouble(minTimeStr[i]);
            } catch (NumberFormatException e) {
                throw new IllegalArgumentException("min time string can only contain numbers and commas. " + customizable.minTimes + " is not valid");
            }
        }

        Pose2d shootPose;
        Pose2d preloadShootPose = getSetupPose(customizable.collectionOrder.substring(0, 2));
        for(int i=0; i<numPaths; i++) {
            if(i < numPaths - 1)
                shootPose = getSetupPose(customizable.collectionOrder.substring(i*2 + 2, i*2 + 4));
            else
                shootPose = getSetupPose(customizable.collectionOrder.charAt(i * 2 + 2) + "" + customizable.collectionOrder.charAt(i * 2 + 1));

            String curLetter = customizable.collectionOrder.charAt(i*2+1) + "";
            boolean fromNear = customizable.collectionOrder.charAt(i*2) == 'n';
            boolean toNear = customizable.collectionOrder.charAt(i*2+2) == 'n';
            double minTime = minTimes[i+1];
            telemetry.addLine("Path " + (i+1) + ": letter: " + curLetter + " from near: " + fromNear + " to near: " + toNear + " min time: " + minTime);

            switch(curLetter) {
                case "1" :
                    actionOrder.add(getFirstCollectAndShoot(shootPose, fromNear, toNear, minTime));
                    break;
                case "2" :
                    actionOrder.add(getSecondCollectAndShoot(shootPose, fromNear, toNear, minTime));
                    break;
                case "3" :
                    actionOrder.add(getThirdCollectAndShoot(shootPose, fromNear, minTime));
                    break;
                case "l" : actionOrder.add(getLoadingCollectAndShoot(shootPose, minTime)); break;
                case "c": actionOrder.add(getRepeatedCornerCollectAndShoot(shootPose, fromNear, minTime)); break;
                case "g": actionOrder.add(getGateCollectAndShoot(shootPose, minTime)); break;
            }
        }

        Action autoAction = new SequentialAction(
                getPreloadDriveAndShoot(preloadShootPose, minTimes[0]),
                actionOrder.get(0),
                numPaths > 1 ? actionOrder.get(1) : new SleepAction(0),
                numPaths > 2 ? actionOrder.get(2) : new SleepAction(0),
                numPaths > 3 ? actionOrder.get(3) : new SleepAction(0),
                numPaths > 4 ? actionOrder.get(4) : new SleepAction(0),
                numPaths > 5 ? actionOrder.get(5) : new SleepAction(0),
                numPaths > 6 ? actionOrder.get(6) : new SleepAction(0),
                numPaths > 7 ? actionOrder.get(7) : new SleepAction(0),
                numPaths > 8 ? actionOrder.get(8) : new SleepAction(0),
                autoCommands.stopIntake(),
                autoCommands.stopShooter()
        );
        Action timedAutoAction = new SequentialAction(
                new CustomEndAction(autoAction,
                        () -> autoTimer.seconds() > timeConstraints.parkStartTime && autoState != AutoState.DRIVE_TO_COLLECT && customizable.useParkAbort, 500),
                decideParkDrive(),
                autoCommands.stopIntake(),
                autoCommands.stopShooter()
        );

        Action forcedStopAutoAction = new ParallelAction(
                packet -> { telemetry.addData("AUTO STATE", autoState); return true; },
                new TimedAction(timedAutoAction, timeConstraints.stopEverythingTime).setEndFunction(robot.drive::stop),
                autoCommands.updateRobot,
                autoCommands.savePoseContinuously,
                packet -> {
                    telemetry.addData("TIMER", autoTimer.seconds());
                    telemetry.addData("current", robot.collection.collectorMotor.getCurrent(CurrentUnit.MILLIAMPS));
                    telemetry.addData("balls shot", robot.shooter.getBallsShot());
                    telemetry.addData("intake p", robot.collection.getIntakePower());
                    telemetry.addData("shooter error", robot.shooter.shooterPID.getTarget() - robot.shooter.getAvgMotorVelocity());
                    telemetry.addData("autoX, y, heading", PoseStorage.autoX + ", " + PoseStorage.autoY + ", " + Math.floor(PoseStorage.autoHeading * 180 / Math.PI));
                    telemetry.update();
                    return true;
                }
        );
        robot.turret.resetEncoders();

        telemetry.addData("alliance", alliance);
        telemetry.addData("auto string", customizable.collectionOrder);
        telemetry.addLine("READY TO RUN");
        telemetry.update();
        waitForStart();
        autoTimer.reset();

        robot.shooter.setBallsShot(0); // always start with 3 preloads

        Actions.runBlocking(
                forcedStopAutoAction

        );
        robot.drive.stop();
    }
    private Pose2d getSetupPose(String info) {
        boolean shootClose = info.charAt(0) == 'n';
        String letter = info.charAt(1) + "";
        switch (letter) {
            case "1": return shootClose ? shootNearSetup1Pose : shootFarSetup1Pose;
            case "2":
            case "g": return shootClose ? shootNearSetup2Pose : shootFarSetup2Pose;
            case "3": return shootClose ? shootNearSetup3Pose : shootFarSetup3Pose;
            case "l": return shootClose ? shootNearSetupLoadingPose : shootFarSetupLoadingPose;
            default: throw new IllegalArgumentException("invalid collectionOrder of " + customizable.collectionOrder + "; can only contain 1, 2, 3, L/l, or G/g");
        }
    }

    private Action getPreloadDriveAndShoot(Pose2d shootPose, double minTime) {
        DrivePath preloadShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3));
        return new SequentialAction(
                new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
                new ParallelAction(
                        autoCommands.speedUpShooter(),
                        autoCommands.enableTurretTracking(),
                        preloadShootDrive
                ),
                new InstantAction(() -> autoState = AutoState.SHOOT),
                waitUntilMinTime(minTime),
                autoCommands.engageClutch(),
                autoCommands.runIntake(),
                autoCommands.flickerHalfUp(),
                new SequentialAction(
                        new SleepAction(timeConstraints.minShootTime),
                        autoCommands.waitTillDoneShooting(Collection.COLLECTOR_PARAMS.maxTimeBetweenShots, Collection.COLLECTOR_PARAMS.hasBallValidationTime)
                ),
                decideFlicker(),
                autoCommands.disengageClutch()
        );
    }
    private Action buildCollectAndShoot(Action collectDrive, Action gateDrive, Action shootDrive, double minTime, double postIntakeTime) {
        return new SequentialAction(
                autoCommands.runIntake(),
                new InstantAction(() -> autoState = AutoState.DRIVE_TO_COLLECT),
                collectDrive,
                new ParallelAction(
                        new SequentialAction(
                                new InstantAction(() -> autoState = AutoState.OPEN_GATE),
                                gateDrive,
                                new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
                                shootDrive,
                                packet -> {robot.drive.stop(); return false; }
                        ),
                        new SequentialAction(
                                new SleepAction(postIntakeTime),
                                autoCommands.stopIntake()
                        )
                ),
                new InstantAction(() -> autoState = AutoState.SHOOT),
                autoCommands.speedUpShooter(),
                waitUntilMinTime(minTime),
                telemetryPacket -> {
                    robot.shooter.setBallsShot(0);
                    return false;
                },
                autoCommands.runIntake(),
                autoCommands.engageClutch(),
                autoCommands.flickerHalfUp(),
                new SleepAction(timeConstraints.minShootTime),
                autoCommands.waitTillDoneShooting(Collection.COLLECTOR_PARAMS.maxTimeBetweenShots, Collection.COLLECTOR_PARAMS.hasBallValidationTime),
                decideFlicker(),
                autoCommands.disengageClutch()
        );
    }
    private Action getFirstCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, double minTime) {
        DrivePath firstCollectDrive;
        if(fromNear)
            firstCollectDrive = new DrivePath(robot.drive, telemetry, new Waypoint(collect1Pose)
                    .setMinLinearPower(collect.collectDrivePower)
                    .setMaxLinearPower(collect.collectDrivePower)
                    .setMaxTime(1.4)
                    .setCustomEndCondition(() -> robot.collection.intakeHas3Balls()));
        else {
            Pose2d preCollectPose = new Pose2d(preCollect1Pose.position.x, preCollect1Pose.position.y, preCollect1Pose.heading.toDouble());
            firstCollectDrive = new DrivePath(robot.drive, telemetry,
                    new Waypoint(preCollectPose, collect.waypointTol)
                            .setPassPosition(true)
                            .setSlowDownPercent(collect.waypointSlowDown)
                            .setMaxTime(2)
                            .setHeadingLerp(PathParams.HeadingLerpType.TANGENT),
                    new Waypoint(collect1Pose)
                            .setPassPosition(true)
                            .setMinLinearPower(collect.collectDrivePower)
                            .setMaxLinearPower(collect.collectDrivePower)
                            .setMaxTime(2)
                            .setCustomEndCondition(() -> robot.collection.intakeHas3Balls()));
        }

        double sign = isRed ? -1 : 1;
        Pose2d preGate1Pose = new Pose2d(gate1Pose.position.x + misc.preGateXOffset, gate1Pose.position.y + sign * misc.preGateClearance, gate1Pose.heading.toDouble());

        Action firstGateDrive = customizable.openGateOnFirst ?
                new SequentialAction(
                        new DrivePath(robot.drive, telemetry,
                                new Waypoint(preGate1Pose, misc.preGateTol).setMinLinearPower(misc.gateMinPower).setPassPosition(true),
                                new Waypoint(gate1Pose).setMinLinearPower(misc.gateMinPower)),
                        new SleepAction(timeConstraints.gateOpeningWait)
                )
                : new SleepAction(0);
        Action firstShootDrive = toNear ?
                new DrivePath(robot.drive, new Waypoint(shootPose)
                        .setMaxTime(3).setPassPosition(true))
                :
                new DrivePath(robot.drive, telemetry,
                        new Waypoint(shootFar1WaypointPose, shoot.waypointTol)
                                .setPassPosition(true)
                                .setSlowDownPercent(shoot.waypointSlowDown)
                                .setMaxTime(10),
                        new Waypoint(shootPose)
                                .setMaxTime(10).setPassPosition(true));

        return buildCollectAndShoot(firstCollectDrive, firstGateDrive, firstShootDrive, minTime, timeConstraints.postIntakeTime);
    }
    private Action getSecondCollectAndShoot(Pose2d shootPose, boolean fromNear, boolean toNear, double minTime) {
        double xOffset = fromNear ? -collect.preCollect2NearXOffset : collect.preCollect2NearXOffset;
        Pose2d preCollectPose = new Pose2d(preCollect2Pose.position.x + xOffset, preCollect2Pose.position.y, preCollect2Pose.heading.toDouble());

        Action secondCollectDrive = new DrivePath(robot.drive, telemetry,
                new Waypoint(preCollectPose, collect.waypointTol)
                        .setPassPosition(true)
                        .setHeadingLerp(PathParams.HeadingLerpType.TANGENT)
                        .setSlowDownPercent(collect.waypointSlowDown)
                        .setMaxTime(2),
                new Waypoint(collect2Pose)
                        .setPassPosition(true)
                        .setMinLinearPower(collect.collectDrivePower)
                        .setMaxLinearPower(collect.collectDrivePower)
                        .setMaxTime(2)
                        .setCustomEndCondition(() -> robot.collection.intakeHas3Balls())
                );

        double sign = isRed ? -1 : 1;
        Pose2d preGatePose = new Pose2d(gate2Pose.position.x - misc.preGateXOffset, gate2Pose.position.y + sign * misc.preGateClearance, gate2Pose.heading.toDouble());

        Action secondGateDrive = customizable.openGateOnSecond ?
                new SequentialAction(
                        new DrivePath(robot.drive, telemetry,
                                new Waypoint(preGatePose, misc.preGateTol).setMinLinearPower(misc.gateCollectWaypointMinPower).setPassPosition(true).setSlowDownPercent(0).setMaxTime(0.5),
                                new Waypoint(gate2Pose).setMinLinearPower(misc.gateMinPower)),
                        new SleepAction(timeConstraints.gateOpeningWait)
                )
                : new SleepAction(0);

        DrivePath secondShootDrive;
        if(toNear)
            secondShootDrive = new DrivePath(robot.drive, telemetry,
                    new Waypoint(shootPose).setMaxTime(3).setPassPosition(true));
        else
            secondShootDrive = new DrivePath(robot.drive,
                    new Waypoint(shootFar2WaypointPose)
                            .setMaxTime(2)
                            .setSlowDownPercent(shoot.waypointSlowDown),
                    new Waypoint(shootPose)
                            .setMaxTime(2).setPassPosition(true));

        return buildCollectAndShoot(secondCollectDrive, secondGateDrive, secondShootDrive, minTime, timeConstraints.postIntakeTime);
    }
    private Action getThirdCollectAndShoot(Pose2d shootPose, boolean fromNear, double minTime) {
        double xOffset = fromNear ? -collect.preCollect3NearXOffset : collect.preCollect2NearXOffset;
        Pose2d preCollectPose = new Pose2d(preCollect3Pose.position.x + xOffset, preCollect3Pose.position.y, preCollect3Pose.heading.toDouble());

        DrivePath thirdCollectDrive = new DrivePath(robot.drive, telemetry,
                new Waypoint(preCollectPose, collect.waypointTol)
                        .setPassPosition(true)
                        .setMaxTime(3)
                        .setHeadingLerp(PathParams.HeadingLerpType.TANGENT)
                        .setSlowDownPercent(collect.waypointSlowDown),
                new Waypoint(collect3Pose)
                        .setMaxTime(2).setPassPosition(true)
                        .setMinLinearPower(collect.collectDrivePower).setMaxLinearPower(collect.collectDrivePower)
                        .setCustomEndCondition(() -> robot.collection.intakeHas3Balls()));

        Action thirdShootDrive = new DrivePath(robot.drive, telemetry, new Waypoint(shootPose)
                .setMaxTime(3).setPassPosition(true));

        return buildCollectAndShoot(thirdCollectDrive, new SleepAction(0), thirdShootDrive, minTime, timeConstraints.postIntakeTime);
    }
    private Action getLoadingCollectAndShoot(Pose2d shootPose, double minTime) {
        DrivePath loadingCollectDrive = new DrivePath(robot.drive,
                new Waypoint(preLoadingPose)
                        .setMaxTime(2),
                new Waypoint(postLoadingPose)
                        .setMinLinearPower(collect.collectDrivePower).setMaxLinearPower(collect.collectDrivePower)
                        .setMaxTime(3)
                        .setCustomEndCondition(() -> robot.collection.intakeHas3Balls()));
        DrivePath loadingShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3));

        return buildCollectAndShoot(loadingCollectDrive, new SleepAction(0), loadingShootDrive, minTime, timeConstraints.loadingSlowIntakeTime);
    }

    private Action getRepeatedCornerCollectAndShoot(Pose2d shootPose, boolean fromNear, double minTime) {
        DrivePath gateShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(4));

        return buildCollectAndShoot(repeatedCornerCollect(fromNear), new SleepAction(0), gateShootDrive, minTime, timeConstraints.loadingSlowIntakeTime);
    }
    private Action repeatedCornerCollect(boolean fromNear) {
        return new Action() {
            private Action gateCollectDrive, gateResetDrive;
            private boolean first = true;
            private boolean ranCollectDrive = false;
            private int numTimesCollected = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    first = false;
                    resetCollectDrive();
                }
                if (!ranCollectDrive) {
                    if (!gateCollectDrive.run(telemetryPacket)) {
                        ranCollectDrive = true;
                        numTimesCollected++;
                        if (robot.collection.intakeHas3Balls() || numTimesCollected > customizable.maxCornerRetries)
                            return false;
                        resetRetryDrive();
                        resetCollectDrive();
                    }
                }
                if (ranCollectDrive) {
                    ranCollectDrive = gateResetDrive.run(telemetryPacket);
                }
                return true;
            }
            private void resetCollectDrive() {
                if (numTimesCollected == 0) {
                    double sign = isRed ? 1 : -1;
                    double collectTangent = fromNear ? sign * Math.toRadians(30) : sign * Math.toRadians(80);
                    gateCollectDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
                            .setTangent(collectTangent)
                            .splineToSplineHeading(cornerCollectPose, collectTangent)
                            .build(),
                            () -> robot.collection.intakeHas3Balls(), timeConstraints.cornerCollectMaxTime);
                }
                else {
                    gateCollectDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
                            .strafeToLinearHeading(new Vector2d(collect.cornerCollectRetryX, cornerCollectPose.position.y), cornerCollectPose.heading.toDouble())
                            .build(),
                            () -> robot.collection.intakeHas3Balls(), timeConstraints.cornerCollectMaxTime);
                }
            }
            private void resetRetryDrive() {
                gateResetDrive = new CustomEndAction(robot.drive.actionBuilder(robot.drive.localizer.getPose())
                        .strafeToLinearHeading(cornerCollectRetryPose.position, cornerCollectRetryPose.heading.toDouble())
                        .build(),
                        () -> robot.collection.intakeHas3Balls() || Math.abs(robot.drive.localizer.getPose().position.y) < (cornerCollectRetryPose.position.y) + 1);
            }
        };
    }

    private Action getGateCollectAndShoot(Pose2d shootPose, double minTime) {
        Action gateCollectDrive = new SequentialAction(
                autoCommands.stopIntake(),
                new DrivePath(robot.drive,
                        new Waypoint(gateCollectWaypointPose, collect.gateCollectWaypointTol).setSlowDownPercent(0).setPassPosition(true).setMinLinearPower(collect.collectDrivePower),
                        new Waypoint(gateCollectOpenPose).setMaxTime(1.5)),
                autoCommands.runIntake(),
                new DrivePath(robot.drive,
//                        new Waypoint(gateCollectBackupPose, misc.preGateTol).setMaxTime(0.1).setMinLinearPower(misc.gateMinPower).setPassPosition(true),
                        new Waypoint(gateCollectPose).setMaxTime(1.8).setMinLinearPower(misc.gateMinPower).setPassPosition(true)),
                new CustomEndAction(new SleepAction(timeConstraints.gateCollectMaxTime), () -> robot.collection.intakeHas3Balls())
        );

        Action gateShootDrive = new DrivePath(robot.drive,
                new Waypoint(new Pose2d(gateCollectWaypointPose.position, shootPose.heading.toDouble()), collect.waypointTol).setSlowDownPercent(0).setPassPosition(true),
                new Waypoint(shootPose).setMaxTime(1.4).setPassPosition(true));

        return buildCollectAndShoot(gateCollectDrive, new SleepAction(0), gateShootDrive, minTime, timeConstraints.postIntakeTime);

    }

    private Action waitUntilMinTime(double minTime) {
        return packet -> minTime > 0 && autoTimer.seconds() < minTime;
    }
    private Action decideFlicker() {
        return new Action() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean first = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (robot.shooter.getBallsShot() == 3)
                    return false;

                if (first) {
                    first = false;
                    timer.reset();
                }

                robot.collection.setFlickerState(Collection.FlickerState.FULL_UP_DOWN);
                robot.collection.setCollectionState(Collection.CollectionState.OFF);

                return timer.seconds() < 0.4;
            }
        };
    }
    private Action decideParkDrive() {
        return new Action() {
            private boolean first = true;
            private Action parkDrive;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (autoState == AutoState.DRIVE_TO_COLLECT) {
                    robot.drive.stop();
                    return false;
                }

                if (first) {
                    first = false;
                    declareDrive();
                }

                return parkDrive.run(telemetryPacket);
            }
            private void declareDrive() {
                boolean isLastShootPoseNear = customizable.collectionOrder.charAt(customizable.collectionOrder.length() - 1) == 'n';
                Pose2d startPose = robot.drive.localizer.getPose();
                Pose2d parkPose = isLastShootPoseNear ? parkNearPose : parkFarPose;
                parkDrive = robot.drive.actionBuilder(startPose)
                        .strafeToConstantHeading(parkPose.position)
                        .build();
            }
        };
    }
    private void declareShootPoses() {
        shootNearSetup1Pose = isRed ? shootNearRed(shoot.shootNearSetup1ARed) : shootNearBlue(shoot.shootNearSetup1ABlue);
        shootFarSetup1Pose = isRed ? shootFarRed(shoot.shootFarSetup1ARed) : shootFarBlue(shoot.shootFarSetup1ABlue);
        shootNearSetup2Pose = isRed ? shootMidRed(shoot.shootNearSetup2ARed) : shootMidBlue(shoot.shootNearSetup2ABlue);
        shootFarSetup2Pose = isRed ? shootFarRed(shoot.shootFarSetup2ARed) : shootFarBlue (shoot.shootFarSetup2ABlue);
        shootNearSetup3Pose = isRed ? shootMidRed(shoot.shootNearSetup3ARed) : shootMidBlue(shoot.shootNearSetup3ABlue);
        shootFarSetup3Pose = isRed ? shootFarRed(shoot.shootFarSetup3ARed) : shootFarBlue(shoot.shootFarSetup3ABlue);
        shootNearSetupLoadingPose = isRed ? shootNearRed(shoot.shootNearSetupLoadingARed) : shootNearBlue(shoot.shootNearSetupLoadingABlue);
        shootFarSetupLoadingPose = isRed ? shootFarRed(shoot.shootFarSetupLoadingARed) : shootFarBlue(shoot.shootFarSetupLoadingABlue);

        shootFar1WaypointPose = isRed ? new Pose2d(shoot.shootFar1WaypointXRed, shoot.shootFar1WaypointYRed, shoot.shootFar1WaypointARed) : new Pose2d(shoot.shootFar1WaypointXBlue, shoot.shootFar1WaypointYBlue, shoot.shootFar1WaypointABlue);
        shootFar2WaypointPose = isRed ? new Pose2d(shoot.shootFar2WaypointXRed, shoot.shootFar2WaypointYRed, shoot.shootFar2WaypointARed) : new Pose2d(shoot.shootFar2WaypointXBlue, shoot.shootFar2WaypointYBlue, shoot.shootFar2WaypointABlue);
        shoot2NearWaypointPose = isRed ? new Pose2d(shoot.shoot2NearWaypointXRed, shoot.shoot2NearWaypointYRed, shoot.shoot2NearWaypointARed) : new Pose2d(shoot.shoot2NearWaypointXBlue, shoot.shoot2NearWaypointYBlue, shoot.shoot2NearWaypointABlue);
    }
    private void declareCollectPoses() {
        collect1Pose = isRed ? // only 1 collect pose for near (no pre collect pose, only post collect pose)
                new Pose2d(collect.firstXRed, collect.postFirstYRed, collect.lineARed) :
                new Pose2d(collect.firstXBlue, collect.postFirstYBlue, collect.lineABlue);
        preCollect1Pose = isRed ?
                new Pose2d(collect.firstXRed, collect.preFirstYRed, collect.lineARed) :
                new Pose2d(collect.firstXBlue, collect.preFirstYBlue, collect.lineABlue);

        preCollect2Pose = isRed ?
                new Pose2d(collect.secondXRed, collect.preSecondYRed, collect.lineARed) :
                new Pose2d(collect.secondXBlue, collect.preSecondYBlue, collect.lineABlue);
        collect2Pose = isRed ?
                new Pose2d(collect.secondXRed, collect.postSecondYRed, collect.lineARed) :
                new Pose2d(collect.secondXBlue, collect.postSecondYBlue, collect.lineABlue);

        preCollect3Pose = isRed ?
                new Pose2d(collect.thirdXRed, collect.preThirdYRed, collect.preCollect3NearARed) :
                new Pose2d(collect.thirdXBlue, collect.preThirdYBlue, collect.preCollect3NearABlue);
        collect3Pose = isRed ?
                new Pose2d(collect.thirdXRed, collect.postThirdYRed, collect.lineARed) :
                new Pose2d(collect.thirdXBlue, collect.postThirdYBlue, collect.lineABlue);

        preLoadingPose = isRed ?
                new Pose2d(collect.preLoadingXRed, collect.preLoadingYRed, collect.preLoadingARed) :
                new Pose2d(collect.preLoadingXBlue, collect.preLoadingYBlue, collect.preLoadingABlue);
        postLoadingPose = isRed ?
                new Pose2d(collect.postLoadingXRed, collect.postLoadingYRed, collect.postLoadingARed) :
                new Pose2d(collect.postLoadingXBlue, collect.postLoadingYBlue, collect.postLoadingABlue);
        cornerCollectPose = isRed ?
                new Pose2d(collect.cornerCollectXRed, collect.cornerCollectYRed, collect.cornerCollectARed) :
                new Pose2d(collect.cornerCollectXBlue, collect.cornerCollectYBlue, collect.cornerCollectABlue);
        cornerCollectRetryPose = isRed ?
                new Pose2d(collect.cornerCollectXRed, collect.cornerCollectRetryYRed, collect.cornerCollectARed) :
                new Pose2d(collect.cornerCollectXBlue, collect.cornerCollectRetryYBlue, collect.cornerCollectABlue);

        gateCollectWaypointPose = isRed ?
                new Pose2d(collect.gateCollectWaypointXRed, collect.gateCollectWaypointYRed, Math.toRadians(90)) :
                new Pose2d(collect.gateCollectWaypointXBlue, collect.gateCollectWaypointYBlue, Math.toRadians(-90));
        gateCollectOpenPose = isRed ?
                new Pose2d(collect.gateCollectOpenXRed, collect.gateCollectOpenYRed, collect.gateCollectOpenARed) :
                new Pose2d(collect.gateCollectOpenXBlue, collect.gateCollectOpenYBlue, collect.gateCollectOpenABlue);
        gateCollectPose = isRed ?
                new Pose2d(collect.gateCollectXRed, collect.gateCollectYRed, collect.gateCollectARed) :
                new Pose2d(collect.gateCollectXBlue, collect.gateCollectYBlue, collect.gateCollectABlue);
        gateCollectBackupPose = isRed ?
                new Pose2d(collect.gateCollectBackupXRed, collect.gateCollectBackupYRed, collect.gateCollectOpenARed) :
                new Pose2d(collect.gateCollectBackupXBlue, collect.gateCollectBackupYBlue, collect.gateCollectOpenABlue);
    }
    private void declareMiscPoses() {
        gate1Pose = isRed ?
                new Pose2d(misc.gate1XRed, misc.gateYRed,misc.gateARed) :
                new Pose2d(misc.gate1XBlue, misc.gateYBlue,misc.gateABlue);
        gate2Pose = isRed ?
                new Pose2d(misc.gate2XRed, misc.gateYRed, misc.gateARed) :
                new Pose2d(misc.gate2XBlue, misc.gateYBlue, misc.gateABlue);

        parkNearPose = isRed ?
                new Pose2d(misc.parkNearXRed, misc.parkNearYRed, misc.parkNearARed) :
                new Pose2d(misc.parkNearXBlue, misc.parkNearYBlue, misc.parkNearABlue);
        parkFarPose = isRed ?
                new Pose2d(misc.parkFarXRed, misc.parkFarYRed, misc.parkFarARed) :
                new Pose2d(misc.parkFarXBlue, misc.parkFarYBlue, misc.parkFarABlue);
    }
    public Pose2d shootNearRed(double angleRad) { return new Pose2d(shoot.shootNearXRed, shoot.shootNearYRed, angleRad); }
    public Pose2d shootMidRed(double angleRad) { return new Pose2d(shoot.shootMidXRed, shoot.shootMidYRed, angleRad); };
    public Pose2d shootFarRed(double angleRad) { return new Pose2d(shoot.shootFarXRed, shoot.shootFarYRed, angleRad); }
    public Pose2d shootNearBlue(double angleRad) { return new Pose2d(shoot.shootNearXBlue, shoot.shootNearYBlue, angleRad); }
    public Pose2d shootMidBlue(double angleRad) { return new Pose2d(shoot.shootMidXBlue, shoot.shootMidYBlue, angleRad); };
    public Pose2d shootFarBlue(double angleRad) { return new Pose2d(shoot.shootFarXBlue, shoot.shootFarYBlue, angleRad); }

    public double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    public double launchLineY(double x) {
        return isRed ? -x : x;
    }
}
