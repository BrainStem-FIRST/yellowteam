package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.opmode.testing.PosePredictionErrorRecorder;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Parking;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMath;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.math.HeadingCorrect;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryHelper;

@Config
public abstract class BrainSTEMTeleOp extends LinearOpMode {
    public static boolean showRelative = false, showGlobal = true;
    public static double velocitySize = 10;
    public enum PosePredictType {
        SIMPLE,
        ADVANCED,
        CONTROL
    }
    public static PosePredictType posePredictType = PosePredictType.SIMPLE;
    public static double timeAheadToPredict = 0.075; // if this is -1, it will predict future pose next frame

    BrainSTEMRobot robot;

    GamepadTracker gp1;
    GamepadTracker gp2;

    private Pose2d lastFrameSimplePrediction, lastFrameAdvancedPrediction;
    private final Alliance alliance;
    private boolean currentlyMoving;
    public BrainSTEMTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        lastFrameSimplePrediction = PoseStorage.currentPose;
        lastFrameAdvancedPrediction = PoseStorage.currentPose;
        currentlyMoving = false;

        telemetry.setMsTransmissionInterval(20); // faster telemetry speed
        CommandScheduler.getInstance().reset();

        Pose2d startPose = PoseStorage.currentPose;
        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, startPose); //take pose from auto
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);
        robot.setG1(gp1);
        telemetry.addData("starting pose", startPose.position.x + ", " + startPose.position.y + " | " + startPose.heading.toDouble());
        telemetry.update();

        waitForStart();
        int framesRunning = 0;
        long startTimeNano = System.nanoTime();
        while (opModeIsActive()) {
            gp1.update();
            gp2.update();

            updateDrive();
            updateDriver2();
            updateDriver1();
            CommandScheduler.getInstance().run();
            robot.limelight.printInfo();
            robot.turret.printInfo();
            robot.shooter.printInfo();

            robot.update(currentlyMoving);

            updateDashboardField();

            // print delta time
            framesRunning++;
            double timeRunning = (System.nanoTime() - startTimeNano) * 1.0 * 1e-9;
            if(gp1.isFirstStart()) {
                framesRunning = 0;
                startTimeNano = System.nanoTime();
            }
            telemetry.addData("bbl dist", robot.collection.getBackBottomLaserDist());
            telemetry.addData("btl dist", robot.collection.getBackTopLaserDist());
            telemetry.addData("turret pos", robot.limelight.getTurretPos());
            telemetry.addData("turret heading", Math.floor(robot.limelight.getTurretHeading() * 180 / Math.PI));
            telemetry.addData("robot pos", robot.limelight.getRobotPos());
            telemetry.addData("robot heading", Math.floor(robot.limelight.getRobotHeading() * 180 / Math.PI));
            telemetry.addData("FPS", MathUtils.format2(framesRunning / timeRunning));
            telemetry.update();
        }
    }

    private void updateDrive() {
        currentlyMoving = gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0;
        robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));
    }

    private void updateDriver1() {
        if (gp1.isFirstA())
            if (robot.collection.collectionState == Collection.CollectionState.INTAKE)
                robot.collection.collectionState = Collection.CollectionState.OFF;
            else
                robot.collection.collectionState = Collection.CollectionState.INTAKE;

        if (gp1.isFirstB()) {
            if (robot.collection.clutchState == Collection.ClutchState.ENGAGED)
                robot.collection.clutchState = Collection.ClutchState.UNENGAGED;
            else
                robot.collection.clutchState = Collection.ClutchState.ENGAGED;
            robot.collection.clutchStateTimer.reset();
        }

        if (gp1.isFirstY())
            if (robot.shooter.shooterState == Shooter.ShooterState.UPDATE)
                robot.shooter.shooterState = Shooter.ShooterState.OFF;
            else
                robot.shooter.shooterState = Shooter.ShooterState.UPDATE;
        if (gp1.isFirstDpadDown())
            robot.shooter.shooterTrackerCommand(gp1).schedule();

        if (gp1.isFirstX())
            if (robot.turret.turretState == Turret.TurretState.CENTER)
                robot.turret.turretState = Turret.TurretState.TRACKING;
            else
                robot.turret.turretState = Turret.TurretState.CENTER;

        if (gp1.isFirstDpadUp()) {
            if (robot.collection.collectionState == Collection.CollectionState.EXTAKE)
                robot.collection.collectionState = Collection.CollectionState.OFF;
            else
                robot.collection.collectionState = Collection.CollectionState.EXTAKE;
        }

        if (gp1.isFirstRightBumper()) {
            robot.collection.flickerState = Collection.FlickerState.UP_DOWN;
        }

        // TODO: FIGURE OUT LIMELIGHT CONTROLS
        if(gp1.isFirstLeftBumper())
            robot.updatePoseWithLimelight();
    }

    private void updateDriver2() {

        if (gp2.isFirstA())
            if (robot.collection.collectionState == Collection.CollectionState.INTAKE)
                robot.collection.collectionState = Collection.CollectionState.OFF;
            else
                robot.collection.collectionState = Collection.CollectionState.INTAKE;
        if (gp2.isFirstB())
            if (robot.collection.clutchState == Collection.ClutchState.ENGAGED)
                robot.collection.clutchState = Collection.ClutchState.UNENGAGED;
            else
                robot.collection.clutchState = Collection.ClutchState.ENGAGED;

        if (gp2.isFirstDpadLeft())
            robot.turret.adjustment += 10;
        if (gp2.isFirstDpadRight())
            robot.turret.adjustment -= 10;

        if (gp2.isFirstDpadUp())
            robot.shooter.adjustment += 10;
        if (gp2.isFirstDpadDown())
            robot.shooter.adjustment -= 10;

        if (gp2.isFirstRightBumper()) {
            if (alliance == Alliance.RED) {
                PoseStorage.currentPose = new Pose2d(64, -65.5, Math.toRadians(180));
                robot.drive.localizer.setPose(new Pose2d(64, -65.5, Math.toRadians(180)));
            } else {
                PoseStorage.currentPose = new Pose2d(64, 65.5, Math.toRadians(180));
                robot.drive.localizer.setPose(new Pose2d(64, 65.5, Math.toRadians(180)));
            }
        }

        if (gp2.isFirstY()) {
            if (robot.parking.parkState == Parking.ParkState.EXTENDED)
                robot.turret.turretState = Turret.TurretState.PARK;
            else
                robot.parking.parkState = Parking.ParkState.EXTENDED;
        }
    }
    private void updatePosePredict() {
        // show predicted pose on dashboard
        PinpointLocalizer pinpoint = robot.drive.pinpoint();
        Pose2d actualPose = pinpoint.getPose();
        switch (posePredictType) {
            case SIMPLE: TelemetryHelper.sendRobotPoses(actualPose, lastFrameSimplePrediction, pinpoint.lastPose); break;
            case ADVANCED: TelemetryHelper.sendRobotPoses(actualPose, lastFrameAdvancedPrediction, pinpoint.lastPose); break;
            case CONTROL: TelemetryHelper.sendRobotPoses(actualPose, pinpoint.lastPose); break;
        }
        lastFrameSimplePrediction = pinpoint.getNextPoseSimple(timeAheadToPredict == -1 ? pinpoint.getWeightedDt() : timeAheadToPredict);
        lastFrameAdvancedPrediction = pinpoint.getNextPoseAdvanced();

        if (gamepad1.back)
            trackPosePredict(actualPose);
    }
    private void trackPosePredict(Pose2d actualPose) {
        // save simple and advanced errors
        OdoInfo simpleError = new OdoInfo(lastFrameSimplePrediction.position.x - actualPose.position.x,
                lastFrameSimplePrediction.position.y - actualPose.position.y,
                HeadingCorrect.correctHeadingErrorRad(lastFrameSimplePrediction.heading.toDouble() - actualPose.heading.toDouble()));
        PosePredictionErrorRecorder.predictionErrorsSimple.add(simpleError);
        OdoInfo advancedError = new OdoInfo(lastFrameAdvancedPrediction.position.x - actualPose.position.x,
                lastFrameAdvancedPrediction.position.y - actualPose.position.y,
                HeadingCorrect.correctHeadingErrorRad(lastFrameAdvancedPrediction.heading.toDouble() - actualPose.heading.toDouble()));
        PosePredictionErrorRecorder.predictionErrorsAdvanced.add(advancedError);

        // save current velocity and acceleration
        PinpointLocalizer pinpoint = robot.drive.pinpoint();
        if (!pinpoint.previousAccelerations.isEmpty())
            PosePredictionErrorRecorder.acceleration.add(pinpoint.previousAccelerations.get(0));
        if (!pinpoint.previousVelocities.isEmpty())
            PosePredictionErrorRecorder.velocity.add(pinpoint.previousVelocities.get(0));

        // save control group errors
        Pose2d lastPose = pinpoint.lastPose;
        OdoInfo controlGroupError = new OdoInfo(lastPose.position.x - actualPose.position.x,
                lastPose.position.y - actualPose.position.y,
                lastPose.heading.toDouble() - actualPose.heading.toDouble()
        );
        PosePredictionErrorRecorder.controlGroupError.add(controlGroupError);
    }
    private void updateDashboardField() {
        Pose2d robotPose = robot.drive.pinpoint().getPose();
        Vector2d exitPosition = ShootingMath.calculateExitPositionInches(robotPose, robot.turret.getTurretEncoder(), robot.shooter.getBallExitAngleRad());

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        TelemetryHelper.addRobotPoseToCanvas(fieldOverlay, robotPose, new Pose2d(exitPosition.x, exitPosition.y, robot.turret.targetAngleRad));

        // draw exit position velocity
        fieldOverlay.setAlpha(1);
        fieldOverlay.setStroke("black");
        fieldOverlay.strokeLine(robotPose.position.x,
                robotPose.position.y,
                robotPose.position.x + robot.turret.exitVelocityMps.x,
                robotPose.position.y + robot.turret.exitVelocityMps.y);

        // draw where turret is pointed
        double dist = Math.hypot(exitPosition.x - robot.turret.targetPose.position.x, exitPosition.y - robot.turret.targetPose.position.y);

        fieldOverlay.setStroke("green");
        fieldOverlay.strokeLine(
                exitPosition.x,
                exitPosition.y,
                exitPosition.x + dist * Math.cos(robot.turret.currentAngleRad),
                exitPosition.y + dist * Math.sin(robot.turret.currentAngleRad)
        );
        fieldOverlay.setStroke("blue");
        fieldOverlay.strokeLine(
                exitPosition.x,
                exitPosition.y,
                exitPosition.x + dist * Math.cos(robot.turret.targetAngleRad),
                exitPosition.y + dist * Math.sin(robot.turret.targetAngleRad)
        );

        // draw goal
        fieldOverlay.setStroke("yellow");
        fieldOverlay.strokeCircle(robot.turret.targetPose.position.x, robot.turret.targetPose.position.y, 3);
        Pose2d defaultGoalPose = robot.turret.targetPose;
        fieldOverlay.strokeCircle(defaultGoalPose.position.x, defaultGoalPose.position.y, 3);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
//    private void updateDashboardField() {
//        Pose2d robotPose = robot.drive.pinpoint().getPose();
//        Pose2d turretPose = Turret.getTurretPose(robotPose, robot.turret.getTurretEncoder());
//
//        TelemetryPacket packet = new TelemetryPacket();
//        Canvas fieldOverlay = packet.fieldOverlay();
//        TelemetryHelper.addRobotPoseToCanvas(fieldOverlay, robotPose, turretPose);
//        fieldOverlay.setAlpha(1);
//        fieldOverlay.setStroke("black");
//        fieldOverlay.strokeLine(robotPose.position.x,
//                robotPose.position.y,
//                robotPose.position.x + robot.turret.exitVelocityMps.x,
//                robotPose.position.y + robot.turret.exitVelocityMps.y);
//        if (showRelative) {
//            Vec vec = robot.turret.relativeBallExitVelocityMps.normalize().mult(velocitySize);
//            fieldOverlay.setStroke("green");
//            fieldOverlay.strokeLine(turretPose.position.x,
//                    turretPose.position.y,
//                    turretPose.position.x + vec.x,
//                    turretPose.position.y + vec.y
//            );
//        }
//        if (showGlobal) {
//            Vec vec = robot.turret.globalBallExitVelocityMps.normalize().mult(velocitySize);
//            fieldOverlay.setStroke("blue");
//            fieldOverlay.strokeLine(turretPose.position.x,
//                    turretPose.position.y,
//                    turretPose.position.x + vec.x,
//                    turretPose.position.y + vec.y
//            );
//        }
//
//        fieldOverlay.setStroke("yellow");
//        fieldOverlay.strokeCircle(robot.turret.targetPose.position.x, robot.turret.targetPose.position.y, 5);
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
//
//       TelemetryHelper.sendRobotPoses(robotPose, robot.turret.getTurretPose(robotPose));
//    }

}