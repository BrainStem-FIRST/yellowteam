package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterState.REVERSE_FULL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.opmode.testing.PosePredictionErrorRecorder;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Parking;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMath;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.math.HeadingCorrect;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryHelper;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;

import java.util.List;

@Config
public class Tele extends OpMode {
    public static boolean ranAutoAsLastOpMode = false;
    public enum PosePredictType {
        SIMPLE,
        ADVANCED,
        CONTROL
    }
    public static class PosePredictParams {
        public PosePredictType posePredictType = PosePredictType.SIMPLE;
        public double timeAheadToPredict = 0.075; // if this is -1, it will predict future pose next frame
    }
    public static PosePredictParams posePredictParams = new PosePredictParams();
    public static class TelemetryParams {
        public boolean showCollector = false, showLimelight = false, showShooter = false, showTurret = false, showParking = false;
    }
    public static TelemetryParams telemetryParams = new TelemetryParams();
    public static LynxModule.BulkCachingMode bulkCachingMode = LynxModule.BulkCachingMode.OFF;

    BrainSTEMRobot robot;

    GamepadTracker gp1;
    GamepadTracker gp2;

    private Pose2d lastFrameSimplePrediction, lastFrameAdvancedPrediction;
    private final Alliance alliance;
    private boolean currentlyMoving;
    private List<LynxModule> allHubs;

    private int framesRunning;
    private long startTimeNano;
    public Tele(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        if (ranAutoAsLastOpMode) {
            PoseStorage.autoX = 0;
            PoseStorage.autoY = 0;
            PoseStorage.autoHeading = 0;
        }
        ranAutoAsLastOpMode = false;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs)
            hub.setBulkCachingMode(bulkCachingMode);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        Pose2d startPose = new Pose2d(PoseStorage.autoX, PoseStorage.autoY, PoseStorage.autoHeading);
        lastFrameSimplePrediction = startPose;
        lastFrameAdvancedPrediction = startPose;
        currentlyMoving = false;

        telemetry.setMsTransmissionInterval(20); // faster telemetry speed
        CommandScheduler.getInstance().reset();

        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, startPose); //take pose from auto
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);
        robot.setG1(gp1);

        telemetry.addData("starting pose", startPose.position.x + ", " + startPose.position.y + " | " + startPose.heading.toDouble());
        telemetry.update();
    }

    @Override
    public void start() {
        framesRunning = 0;
        startTimeNano = System.nanoTime();
    }

    @Override
    public void loop() {
        if (bulkCachingMode == LynxModule.BulkCachingMode.MANUAL)
            for(LynxModule hub : allHubs)
                hub.clearBulkCache();

        gp1.update();
        gp2.update();

        telemetry.addData("TRACKING SHOOTER DATA", robot.shooter.isTrackingData());
        telemetry.addLine();
        telemetry.addData("SHOOTER ADJUSTMENT", robot.shooter.adjustment);
        telemetry.addData("TURRET ADJUSTMENT", robot.turret.adjustment);
        telemetry.addLine();

        updateDrive();
        updateDriver2();
        updateDriver1();
        CommandScheduler.getInstance().run();

        robot.update(currentlyMoving);

        // update telemetry
        updateDashboardField();

        framesRunning++;
        double timeRunning = (System.nanoTime() - startTimeNano) * 1.0 * 1e-9;
        telemetry.addData("bulk caching mode", bulkCachingMode);
        telemetry.addData("FPS", MathUtils.format2(framesRunning / timeRunning));
        telemetry.addData("Loop time (ms)", MathUtils.format2(1000 * timeRunning / framesRunning));

        if (telemetryParams.showCollector)
            robot.collection.printInfo();
        if (telemetryParams.showLimelight)
            robot.limelight.printInfo();
        if (telemetryParams.showShooter)
            robot.shooter.printInfo();
        if (telemetryParams.showTurret)
            robot.turret.printInfo();
        if (telemetryParams.showParking)
            robot.parking.printInfo();

        telemetry.update();

//        Pose2d p = robot.drive.localizer.getPose();
//        PoseStorage.autoX = p.position.x;
//        PoseStorage.autoY = p.position.y;
//        PoseStorage.autoHeading = p.heading.toDouble();
    }

    private double lastLsx, lastLsy, lastRsx;
    private void updateDrive() {
        if (robot.limelight.getState() == Limelight.UpdateState.UPDATING_POSE && robot.limelight.manualPoseUpdate) {
            if (robot.limelight.getPrevState() != Limelight.UpdateState.UPDATING_POSE)
                robot.drive.stop();

            return;
        }

        double lsx = gamepad1.left_stick_x, lsy = gamepad1.left_stick_y, rsx = gamepad1.right_stick_x;
        currentlyMoving = lsx != 0 || lsy != 0 || rsx != 0;

        boolean newInputs = lastLsx != lsx || lastLsy != lsy || lastRsx != rsx;
        if (newInputs) {
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
        }

        lastLsx = gamepad1.left_stick_x;
        lastLsy = gamepad1.left_stick_y;
        lastRsx = gamepad1.right_stick_x;
    }

    private void updateDriver1() {
        if(robot.collection.getClutchState() == Collection.ClutchState.UNENGAGED) {
            if (gp1.gamepad.right_trigger > 0.2)
                robot.collection.setCollectionState(Collection.CollectionState.INTAKE);
            else if (gp1.gamepad.left_trigger > 0.2)
                robot.collection.setCollectionState(Collection.CollectionState.OUTTAKE);
            else
                robot.collection.setCollectionState(Collection.CollectionState.OFF);
        }

        if (gp1.isFirstB()) {
            if (robot.collection.getClutchState() == Collection.ClutchState.ENGAGED)
                robot.collection.setClutchState(Collection.ClutchState.UNENGAGED);
            else
                robot.collection.setClutchState(Collection.ClutchState.ENGAGED);
            robot.collection.clutchStateTimer.reset();
        }

        if (gp1.isFirstRightBumper())
            if (robot.shooter.shooterState == Shooter.ShooterState.UPDATE)
                robot.shooter.shooterState = Shooter.ShooterState.OFF;
            else
                robot.shooter.shooterState = Shooter.ShooterState.UPDATE;

        if (gp1.isFirstDpadDown()) {
            if (robot.shooter.isTrackingData())
                robot.shooter.stopManualTrackingData();
            else
                robot.shooter.startManualTrackingData();
        }
        if (gp1.isFirstLeftBumper()) {
            if (robot.turret.turretState == Turret.TurretState.CENTER)
                robot.turret.turretState = Turret.TurretState.TRACKING;
            else
                robot.turret.turretState = Turret.TurretState.CENTER;
        }

        if (gp1.isFirstBack()) {
            robot.limelight.maxTranslationalError = 0;
            robot.limelight.maxHeadingErrorDeg = 0;
        }
    }

    private void updateDriver2() {
        if(robot.collection.getClutchState() == Collection.ClutchState.ENGAGED) {
            if (gp2.isFirstA())
                if (robot.collection.getCollectionState() == Collection.CollectionState.INTAKE)
                    robot.collection.setCollectionState(Collection.CollectionState.OFF);
                else if (Math.abs(robot.shooter.getAvgMotorVelocity() - robot.shooter.shooterPID.getTarget()) <= Shooter.SHOOTER_PARAMS.firstShootTolerance)
                    robot.collection.setCollectionState(Collection.CollectionState.INTAKE);
        }
        if (gp2.isFirstB())
            if (robot.collection.getClutchState() == Collection.ClutchState.ENGAGED)
                robot.collection.setClutchState(Collection.ClutchState.UNENGAGED);
            else
                robot.collection.setClutchState(Collection.ClutchState.ENGAGED);

        if (gp2.isFirstLeftBumper())
            robot.collection.setFlickerState(Collection.FlickerState.HALF_UP_DOWN);
        if(gp2.isFirstLeftTrigger())
            robot.collection.setFlickerState(Collection.FlickerState.FULL_UP_DOWN);

        if (gp2.isFirstDpadLeft())
            robot.turret.adjustment += Turret.TURRET_PARAMS.fineAdjust;
        if (gp2.isFirstDpadRight())
            robot.turret.adjustment -= Turret.TURRET_PARAMS.fineAdjust;

        if (gp2.isFirstDpadUp())
            robot.shooter.adjustment += 10;
        if (gp2.isFirstDpadDown())
            robot.shooter.adjustment -= 10;

        if (gp2.isFirstY()) {
            if (robot.parking.getParkState() == Parking.ParkState.EXTENDED)
                robot.turret.turretState = Turret.TurretState.PARK;
            else
                robot.parking.setParkState(Parking.ParkState.EXTENDED);
            robot.shooter.shooterState = REVERSE_FULL;
        }

        if(gp2.isFirstRightBumper()) {
            robot.limelight.manualPoseUpdate = true;
            robot.limelight.setState(Limelight.UpdateState.UPDATING_POSE);
        }
        if (gp2.isFirstBack())
            robot.limelight.takePic();
    }
    private void updateDashboardField() {
        Pose2d robotPose = robot.drive.pinpoint().getPose();
        int turretEncoder = robot.turret.getTurretEncoder();
        Pose2d turretPose = Turret.getTurretPose(robotPose, turretEncoder);
        Vector2d exitPosition = ShootingMath.calculateExitPositionInches(robotPose, turretEncoder, robot.shooter.getBallExitAngleRad());
        Pose2d exitPose = new Pose2d(exitPosition, robot.turret.currentAngleRad);
        Pose2d limelightRobotPose = robot.limelight.getRobotPose();
        if (limelightRobotPose == null)
            limelightRobotPose = new Pose2d(0, 0, 0);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        TelemetryHelper.radii[0] = 10;
        TelemetryHelper.radii[1] = 6;
        TelemetryHelper.radii[2] = 3;
        TelemetryHelper.radii[3] = 10;
        TelemetryHelper.numPosesToShow = 4;

        TelemetryHelper.addRobotPoseToCanvas(fieldOverlay, robotPose, turretPose, exitPose, limelightRobotPose);

        // draw where turret is pointed
        fieldOverlay.setAlpha(1);
        double dist = Math.hypot(exitPosition.x - robot.turret.targetPose.position.x, exitPosition.y - robot.turret.targetPose.position.y);

        fieldOverlay.setStroke("purple");
        fieldOverlay.strokeLine(
                exitPosition.x,
                exitPosition.y,
                exitPosition.x + dist * Math.cos(robot.turret.currentAngleRad),
                exitPosition.y + dist * Math.sin(robot.turret.currentAngleRad)
        );
        fieldOverlay.setStroke("black");
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


    private void updatePosePredict() {
        // show predicted pose on dashboard
        PinpointLocalizer pinpoint = robot.drive.pinpoint();
        Pose2d actualPose = pinpoint.getPose();
        switch (posePredictParams.posePredictType) {
            case SIMPLE: TelemetryHelper.sendRobotPoses(actualPose, lastFrameSimplePrediction, pinpoint.lastPose); break;
            case ADVANCED: TelemetryHelper.sendRobotPoses(actualPose, lastFrameAdvancedPrediction, pinpoint.lastPose); break;
            case CONTROL: TelemetryHelper.sendRobotPoses(actualPose, pinpoint.lastPose); break;
        }
        lastFrameSimplePrediction = pinpoint.getNextPoseSimple(posePredictParams.timeAheadToPredict == -1 ? pinpoint.getWeightedDt() : posePredictParams.timeAheadToPredict);
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
}
