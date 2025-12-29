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
    public static class Customizable {
        public String collectionOrder = "n1n2n3n";
        public String minTimes = "-1,-1,-1,-1,-1";
        public boolean openGateOnFirst = true;
        public boolean openGateOnSecond = true;
        public boolean useParkAbort = true;
        public int maxGateRetries = 0;
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
            gateCollectPose,
            gate1Pose, gate2Pose,
            parkNearPose, parkFarPose,
            shootFar1WaypointPose, shootFar2WaypointPose,
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

        Pose2d shootPose = null;
        Pose2d preloadShootPose = getSetupPose(customizable.collectionOrder.substring(0, 2));
        Pose2d prevShootPose = new Pose2d(preloadShootPose.position, preloadShootPose.heading);
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
                    actionOrder.add(getSecondCollectAndShoot(shootPose, toNear, minTime));
                    break;
                case "3" :
                    actionOrder.add(getThirdCollectAndShoot(shootPose, minTime));
                    break;
                case "l" : actionOrder.add(getLoadingCollectAndShoot(shootPose, minTime)); break;
                case "c": actionOrder.add(getRepeatedCornerCollectAndShoot(shootPose, fromNear, minTime)); break;
                case "g": actionOrder.add(getGateCollectAndShoot(shootPose, minTime)); break;
            }
            prevShootPose = new Pose2d(shootPose.position, shootPose.heading);
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
                autoCommands.stopIntake(),
                autoCommands.stopShooter(),
                decideParkDrive()
        );


        Action forcedStopAutoAction = new ParallelAction(
                packet -> { telemetry.addData("AUTO STATE", autoState); return true; },
                new TimedAction(timedAutoAction, timeConstraints.stopEverythingTime).setEndFunction(robot.drive::stop),
                autoCommands.updateRobot,
                autoCommands.savePoseContinuously,
                packet -> {
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
            case "2": return shootClose ? shootNearSetup2Pose : shootFarSetup2Pose;
            case "3": return shootClose ? shootNearSetup3Pose : shootFarSetup3Pose;
            case "l": case "g": return shootClose ? shootNearSetupLoadingPose : shootFarSetupLoadingPose;
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
    private Action buildCollectAndShoot(Action collectDrive, Action gateDrive, Action shootDrive, double minTime, double slowIntakeTime) {
        return new SequentialAction(
                autoCommands.runIntake(),
                new InstantAction(() -> autoState = AutoState.DRIVE_TO_COLLECT),
                collectDrive,
                autoCommands.intakeSlow(),
                new ParallelAction(
                        new SequentialAction(
                                new InstantAction(() -> autoState = AutoState.OPEN_GATE),
                                gateDrive,
                                new InstantAction(() -> autoState = AutoState.DRIVE_TO_SHOOT),
                                shootDrive,
                                packet -> {robot.drive.stop(); return false; }
                        ),
                        new SequentialAction(
                                new SleepAction(slowIntakeTime),
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
            firstCollectDrive = new DrivePath(robot.drive, new Waypoint(collect1Pose)
                    .setMaxPower(collect.collectMaxPower)
                    .setMaxTime(1.4));
        else
            firstCollectDrive = new DrivePath(robot.drive,
                new Waypoint(preCollect1Pose)
                    .setPassPosition()
                    .setSlowDown(collect.waypointSlowDown)
                    .setMaxTime(2)
                    .setTol(collect.waypointTol)
                    .setHeadingLerp(PathParams.HeadingLerpType.TANGENT),
                new Waypoint(collect1Pose)
                        .setPassPosition()
                        .setMaxTime(2));

        Action firstGateDrive = customizable.openGateOnFirst ?
                new SequentialAction(
                        new DrivePath(robot.drive, new Waypoint(gate1Pose)),
                        new SleepAction(timeConstraints.gateOpeningWait)
                )
                : new SleepAction(0);
        Action firstShootDrive = toNear ?
//                robot.drive.actionBuilder(customizable.openGateOnFirst ? gatePose : collectPose)
////                .afterDisp(shoot.clutchDisp1Near, decideEarlyRunIntake(minTime))
//                        .strafeToLinearHeading(shootPose.position, shootPose.heading.toDouble())
//                        .build()
                new DrivePath(robot.drive, new Waypoint(shootPose)
                        .setMaxTime(3))
                :
                new DrivePath(robot.drive,
                        new Waypoint(shootFar1WaypointPose)
                                .setPassPosition()
                                .setSlowDown(shoot.waypointSlowDown)
                                .setMaxTime(10)
                                .setTol(shoot.waypointTol),
                        new Waypoint(shootPose)
                                .setMaxTime(10));
//                robot.drive.actionBuilder(customizable.openGateOnFirst ? gatePose : collectPose)
//                        .setTangent(Math.toRadians(isRed ? -45 : 45))
////                        .afterDisp(shoot.clutchDisp1Far, decideEarlyRunIntake(minTime))
//                        .splineToSplineHeading(shootPose, Math.toRadians(isRed ? -20 : 20))
//                        .build();

        return buildCollectAndShoot(firstCollectDrive, firstGateDrive, firstShootDrive, minTime, timeConstraints.slowIntakeTime);
    }
    private Action getSecondCollectAndShoot(Pose2d shootPose, boolean toNear, double minTime) {

            Action secondCollectDrive = new DrivePath(robot.drive,
                    new Waypoint(preCollect2Pose)
                            .setPassPosition()
                            .setTol(collect.waypointTol)
                            .setHeadingLerp(PathParams.HeadingLerpType.TANGENT)
                            .setSlowDown(collect.waypointSlowDown)
                            .setMaxTime(2),
                    new Waypoint(collect2Pose)
                            .setPassPosition()
                            .setMaxTime(2)
                    );

            Action secondGateDrive = customizable.openGateOnSecond ?
                    new SequentialAction(
                            new DrivePath(robot.drive, new Waypoint(gate2Pose)),
                            new SleepAction(timeConstraints.gateOpeningWait)
                    )
                    : new SleepAction(0);

            DrivePath secondShootDrive;
            if(toNear)
                secondShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                        .setMaxTime(3));
            else
                secondShootDrive = new DrivePath(robot.drive,
                        new Waypoint(shootFar2WaypointPose)
                                .setMaxTime(2)
                                .setSlowDown(shoot.waypointSlowDown),
                        new Waypoint(shootPose)
                                .setMaxTime(2));

            return buildCollectAndShoot(secondCollectDrive, secondGateDrive, secondShootDrive, minTime, timeConstraints.slowIntakeTime);
    }
    private Action getThirdCollectAndShoot(Pose2d shootPose, double minTime) {
        DrivePath thirdCollectDrive = new DrivePath(robot.drive,
                new Waypoint(preCollect3Pose)
                        .setPassPosition()
                        .setMaxTime(3)
                        .setTol(collect.waypointTol)
                        .setHeadingLerp(PathParams.HeadingLerpType.TANGENT)
                        .setSlowDown(collect.waypointSlowDown),
                new Waypoint(collect3Pose)
                        .setMaxTime(2));

        Action thirdShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3));

        return buildCollectAndShoot(thirdCollectDrive, new SleepAction(0), thirdShootDrive, minTime, timeConstraints.slowIntakeTime);
    }
    private Action getLoadingCollectAndShoot(Pose2d shootPose, double minTime) {
        DrivePath loadingCollectDrive = new DrivePath(robot.drive,
                new Waypoint(preLoadingPose)
                        .setMaxTime(2),
                new Waypoint(postLoadingPose)
                        .setMaxTime(3));
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
                        if (robot.collection.intakeHas3Balls() || numTimesCollected > customizable.maxGateRetries)
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
                            .strafeToLinearHeading(new Vector2d(collect.gateCollectRetryX, cornerCollectPose.position.y), cornerCollectPose.heading.toDouble())
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
                new DrivePath(robot.drive, new Waypoint(gateCollectPose)
                        .setMaxTime(4)
                        .setHeadingLerp(PathParams.HeadingLerpType.TANGENT)),
                new SleepAction(timeConstraints.gateCollectMaxTime)
        );

        Action gateShootDrive = new DrivePath(robot.drive, new Waypoint(shootPose)
                .setMaxTime(3));

        return buildCollectAndShoot(gateCollectDrive, new SleepAction(0), gateShootDrive, minTime, timeConstraints.slowIntakeTime);

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
    private Action decideEarlyRunIntake(double minTime) {
        return packet -> {
            if (minTime == -1)
                return autoCommands.runIntake().run(packet);
            return false;
        };
    }
    private void declareShootPoses() {
        shootNearSetup1Pose = isRed ? shootNearRed(shoot.shootNearSetup1ARed) : shootNearBlue(shoot.shootNearSetup1ABlue);
        shootFarSetup1Pose = isRed ? shootFarRed(shoot.shootFarSetup1ARed) : shootFarBlue(shoot.shootFarSetup1ABlue);
        shootNearSetup2Pose = isRed ? shootNearRed(shoot.shootNearSetup2ARed) : shootNearBlue(shoot.shootNearSetup2ABlue);
        shootFarSetup2Pose = isRed ? shootFarRed(shoot.shootFarSetup2ARed) : shootFarBlue (shoot.shootFarSetup2ABlue);
        shootNearSetup3Pose = isRed ? shootNearRed(shoot.shootNearSetup3ARed) : shootNearBlue(shoot.shootNearSetup3ABlue);
        shootFarSetup3Pose = isRed ? shootFarRed(shoot.shootFarSetup3ARed) : shootFarBlue(shoot.shootFarSetup3ABlue);
        shootNearSetupLoadingPose = isRed ? shootNearRed(shoot.shootNearSetupLoadingARed) : shootNearBlue(shoot.shootNearSetupLoadingABlue);
        shootFarSetupLoadingPose = isRed ? shootFarRed(shoot.shootFarSetupLoadingARed) : shootFarBlue(shoot.shootFarSetupLoadingABlue);

        shootFar1WaypointPose = isRed ? new Pose2d(shoot.shootFar1WaypointXRed, shoot.shootFar1WaypointYRed, shoot.shootFar1WaypointARed) : new Pose2d(shoot.shootFar1WaypointXBlue, shoot.shootFar1WaypointYBlue, shoot.shootFar1WaypointABlue);
        shootFar2WaypointPose = isRed ? new Pose2d(shoot.shootFar2WaypointXRed, shoot.shootFar2WaypointYRed, shoot.shootFar2WaypointARed) : new Pose2d(shoot.shootFar2WaypointXBlue, shoot.shootFar2WaypointYBlue, shoot.shootFar2WaypointABlue);
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
                new Pose2d(collect.thirdXRed, collect.preThirdYRed, collect.lineARed) :
                new Pose2d(collect.thirdXBlue, collect.preThirdYBlue, collect.lineABlue);
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

        gateCollectPose = isRed ?
                new Pose2d(collect.gateCollectXRed, collect.gateCollectYRed, collect.gateCollectARed) :
                new Pose2d(collect.gateCollectXBlue, collect.gateCollectYBlue, collect.gateCollectABlue);
    }
    private void declareMiscPoses() {
        gate1Pose = isRed ?
                new Pose2d(misc.gate1XRed, misc.gateYRed, misc.gate1A) :
                new Pose2d(misc.gate1XBlue, misc.gateYBlue, misc.gate1A);
        gate2Pose = isRed ?
                new Pose2d(misc.gate2XRed, misc.gateYRed, misc.gate2A) :
                new Pose2d(misc.gate2XBlue, misc.gateYBlue, misc.gate2A);

        parkNearPose = isRed ?
                new Pose2d(misc.parkNearXRed, misc.parkNearYRed, misc.parkNearARed) :
                new Pose2d(misc.parkNearXBlue, misc.parkNearYBlue, misc.parkNearABlue);
        parkFarPose = isRed ?
                new Pose2d(misc.parkFarXRed, misc.parkFarYRed, misc.parkFarARed) :
                new Pose2d(misc.parkFarXBlue, misc.parkFarYBlue, misc.parkFarABlue);
    }
    public Pose2d shootNearRed(double angleRad) { return new Pose2d(shoot.shootNearXRed, shoot.shootNearYRed, angleRad); }
    public Pose2d shootFarRed(double angleRad) { return new Pose2d(shoot.shootFarXRed, shoot.shootFarYRed, angleRad); }
    public Pose2d shootNearBlue(double angleRad) { return new Pose2d(shoot.shootNearXBlue, shoot.shootNearYBlue, angleRad); }
    public Pose2d shootFarBlue(double angleRad) { return new Pose2d(shoot.shootFarXBlue, shoot.shootFarYBlue, angleRad); }
}
