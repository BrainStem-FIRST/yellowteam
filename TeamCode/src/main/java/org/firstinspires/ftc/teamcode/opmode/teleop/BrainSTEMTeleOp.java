package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.opmode.testing.PosePredictionErrorRecorder;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Parking;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.HeadingCorrect;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.TelemetryHelper;

@Config
public abstract class BrainSTEMTeleOp extends LinearOpMode {
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
    public BrainSTEMTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() {
        lastFrameSimplePrediction = PoseStorage.currentPose;
        lastFrameAdvancedPrediction = PoseStorage.currentPose;

        telemetry.setMsTransmissionInterval(20); // faster telemetry speed
        CommandScheduler.getInstance().reset();

        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, PoseStorage.currentPose); //take pose from auto
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);

        if (Shooter.ENABLE_TESTING) {
            telemetry.addLine("CURRENTLY IN TESTING MODE - SHOOTER VELOCITY SET TO " + Shooter.testingShootVelocity + ", HOOD POSITION SET TO " + Shooter.testingHoodPosition);
            telemetry.update();
        }
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

            robot.update();

            Pose2d robotPose = robot.drive.pinpoint().getPose();
            TelemetryHelper.sendRobotPose(robotPose, robot.turret.getTurretPose(robotPose));
//            updatePosePredict();

            // print delta time
            framesRunning++;
            double timeRunning = (System.nanoTime() - startTimeNano) * 1.0 * 1e-9;
            telemetry.addData("FPS", MathUtils.format2(framesRunning / timeRunning));
            telemetry.addData("predicted dt", MathUtils.format(robot.drive.pinpoint().getWeightedDt(), 6));

            telemetry.update();
        }
    }

    private void updateDrive() {
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

        if (gp1.isFirstB())
            if (robot.collection.clutchState == Collection.ClutchState.ENGAGED)
                robot.collection.clutchState = Collection.ClutchState.UNENGAGED;
            else
                robot.collection.clutchState = Collection.ClutchState.ENGAGED;

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
//
        if (gp1.isFirstDpadUp()) {
            if (robot.collection.collectionState == Collection.CollectionState.EXTAKE)
                robot.collection.collectionState = Collection.CollectionState.OFF;
            else
                robot.collection.collectionState = Collection.CollectionState.EXTAKE;
        }
//
        if (gp1.isFirstRightBumper()) {
            robot.collection.flickerState = Collection.FlickerState.UP_DOWN;
        }

//        if (gp1.isFirstDpadDown())
//            if (robot.shooter.shooterState == Shooter.ShooterState.UPDATE_2)
//                robot.shooter.shooterState = Shooter.ShooterState.OFF;
//            else
//                robot.shooter.shooterState = Shooter.ShooterState.UPDATE_2;


//        if (gp1.isFirstDpadLeft()) {
//            if (parking_position - Parking.PARK_PARAMS.SERVO_INCREMENT >= 0.0)
//                parking_position -= Parking.PARK_PARAMS.SERVO_INCREMENT;
//        }

//        if (gp1.isFirstDpadUp()) {
//            if (hood_position - Shooter.SHOOTER_PARAMS.HOOD_INCREMENT >= 0)
//                hood_position -= Shooter.SHOOTER_PARAMS.HOOD_INCREMENT;
//        }
//
//        if (gp1.isFirstDpadDown()) {
//            if (hood_position + Shooter.SHOOTER_PARAMS.HOOD_INCREMENT <= 1)
//                hood_position += Shooter.SHOOTER_PARAMS.HOOD_INCREMENT;
//        }

//        if (gp1.isFirstDpadRight()) {
//            if (turret_position - Turret.TURRET_PARAMS.TURRET_INCREMENT >= Turret.TURRET_PARAMS.TURRET_MIN)
//                turret_position -= Turret.TURRET_PARAMS.TURRET_INCREMENT;
//        }

//        if (gp1.isFirstDpadLeft()) {
//            if (turret_position + Turret.TURRET_PARAMS.TURRET_INCREMENT <= Turret.TURRET_PARAMS.TURRET_MAX)
//                turret_position += Turret.TURRET_PARAMS.TURRET_INCREMENT;
//        }

//        robot.parking.setParkServoPosition(parking_position);
//        telemetry.addData("** PARKING SERVO POS **", parking_position);

//        robot.shooter.setHoodPosition(hood_position);
//        robot.turret.setTurretPosition(turret_position);
//        telemetry.addData("Turret Increment", turret_position);
//        telemetry.addData("Hood Increment", hood_position);

//        telemetry.addData("FLICKER POS", robot.collection.flickerRight.getPosition());

//        telemetry.addData("Pose X", robot.drive.localizer.getPose().position.x);
//        telemetry.addData("Pose Y", robot.drive.localizer.getPose().position.y);
//        telemetry.addData("Pose Heading", Math.toDegrees(robot.drive.localizer.getPose().heading.toDouble()));
    }

    private void updateDriver2() {
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
            case SIMPLE: TelemetryHelper.sendRobotPose(actualPose, lastFrameSimplePrediction, pinpoint.lastPose); break;
            case ADVANCED: TelemetryHelper.sendRobotPose(actualPose, lastFrameAdvancedPrediction, pinpoint.lastPose); break;
            case CONTROL: TelemetryHelper.sendRobotPose(actualPose, pinpoint.lastPose); break;
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
}