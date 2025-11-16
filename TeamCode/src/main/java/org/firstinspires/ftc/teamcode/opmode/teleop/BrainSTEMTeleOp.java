package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.commandGroups.FullCollectionSequence;
import org.firstinspires.ftc.teamcode.commands.turretCommands.TurretTrackingCommand;
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

@TeleOp(name = "TeleOp", group = "Competition")
@Config
public class BrainSTEMTeleOp extends LinearOpMode {
    public enum PosePredictType {
        SIMPLE,
        ADVANCED,
        CONTROL
    }
    public static PosePredictType posePredictType = PosePredictType.SIMPLE;
    public static double timeAheadToPredict = 0.075; // if this is -1, it will predict future pose next frame

    BrainSTEMRobot brainSTEMRobot;

    // COMMANDS //
    private FullCollectionSequence collectionCommandGroup;
    private TurretTrackingCommand turretTrackingCommand;
    GamepadTracker gp1;
    GamepadTracker gp2;

    // TESTING //
    private double hood_position = 0.1;
    private int turret_position = 0;
    private double parking_position = 0.1;
    private Pose2d lastFrameSimplePrediction, lastFrameAdvancedPrediction;


    @Override
    public void runOpMode() {
        lastFrameSimplePrediction = PoseStorage.currentPose;
        lastFrameAdvancedPrediction = PoseStorage.currentPose;

        telemetry.setMsTransmissionInterval(20); // faster telemetry speed
        CommandScheduler.getInstance().reset();

        brainSTEMRobot = new BrainSTEMRobot(telemetry, hardwareMap, PoseStorage.currentPose); //take pose from auto
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

            telemetry.addData("velocity size", brainSTEMRobot.drive.pinpoint().previousVelocities.size());
            telemetry.addData("accel size", brainSTEMRobot.drive.pinpoint().previousAccelerations.size());

            updateDrive();
//            updateDriver1();
            updateDriver2();
            updateTesting();
            CommandScheduler.getInstance().run();
            brainSTEMRobot.update();
            telemetry.addData("low shooter motor vel", brainSTEMRobot.shooter.shooterMotorLow.getVelocity());
            telemetry.addData("high shooter motor vel", brainSTEMRobot.shooter.shooterMotorHigh.getVelocity());

            updatePosePredict();

            // print delta time
            framesRunning++;
            double timeRunning = (System.nanoTime() - startTimeNano) * 1.0 * 1e-9;
            telemetry.addData("FPS", MathUtils.format2(framesRunning / timeRunning));
            telemetry.addData("predicted dt", MathUtils.format(brainSTEMRobot.drive.pinpoint().getWeightedDt(), 6));

            telemetry.update();
        }

//        brainSTEMRobot.vision.visionPortal.close();
    }

    private void updateDrive() {
        brainSTEMRobot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));
    }

    private void updateTesting() {
        if (gp1.isFirstA())
            if (brainSTEMRobot.collection.collectionState == Collection.CollectionState.INTAKE)
                brainSTEMRobot.collection.collectionState = Collection.CollectionState.OFF;
            else
                brainSTEMRobot.collection.collectionState = Collection.CollectionState.INTAKE;

        if (gp1.isFirstB())
            if (brainSTEMRobot.collection.clutchState == Collection.ClutchState.ENGAGED)
                brainSTEMRobot.collection.clutchState = Collection.ClutchState.UNENGAGED;
            else
                brainSTEMRobot.collection.clutchState = Collection.ClutchState.ENGAGED;

        if (gp1.isFirstY())
            if (brainSTEMRobot.shooter.shooterState == Shooter.ShooterState.UPDATE)
                brainSTEMRobot.shooter.shooterState = Shooter.ShooterState.OFF;
            else
                brainSTEMRobot.shooter.shooterState = Shooter.ShooterState.UPDATE;
        if (gp1.isFirstDpadDown())
            brainSTEMRobot.shooter.shooterTrackerCommand(gp1).schedule();


        if (gp1.isFirstX())
            if (brainSTEMRobot.turret.turretState == Turret.TurretState.CENTER)
                brainSTEMRobot.turret.turretState = Turret.TurretState.TRACKING;
            else
                brainSTEMRobot.turret.turretState = Turret.TurretState.CENTER;
//
        if (gp1.isFirstDpadUp()) {
            if (brainSTEMRobot.collection.collectionState == Collection.CollectionState.EXTAKE)
                brainSTEMRobot.collection.collectionState = Collection.CollectionState.OFF;
            else
                brainSTEMRobot.collection.collectionState = Collection.CollectionState.EXTAKE;
        }
//
        if (gp1.isFirstRightBumper()) {
            brainSTEMRobot.collection.flickerState = Collection.FlickerState.UP_DOWN;
        }

//        if (gp1.isFirstDpadDown())
//            if (brainSTEMRobot.shooter.shooterState == Shooter.ShooterState.UPDATE_2)
//                brainSTEMRobot.shooter.shooterState = Shooter.ShooterState.OFF;
//            else
//                brainSTEMRobot.shooter.shooterState = Shooter.ShooterState.UPDATE_2;


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

//        brainSTEMRobot.parking.setParkServoPosition(parking_position);
//        telemetry.addData("** PARKING SERVO POS **", parking_position);

//        brainSTEMRobot.shooter.setHoodPosition(hood_position);
//        brainSTEMRobot.turret.setTurretPosition(turret_position);
//        telemetry.addData("Turret Increment", turret_position);
//        telemetry.addData("Hood Increment", hood_position);

//        telemetry.addData("FLICKER POS", brainSTEMRobot.collection.flickerRight.getPosition());

//        telemetry.addData("Pose X", brainSTEMRobot.drive.localizer.getPose().position.x);
//        telemetry.addData("Pose Y", brainSTEMRobot.drive.localizer.getPose().position.y);
//        telemetry.addData("Pose Heading", Math.toDegrees(brainSTEMRobot.drive.localizer.getPose().heading.toDouble()));
    }

    private void updateDriver1() {
        if (gp1.isFirstA())
            if (brainSTEMRobot.collection.collectionState == Collection.CollectionState.INTAKE)
                brainSTEMRobot.collection.collectionState = Collection.CollectionState.OFF;
            else
                brainSTEMRobot.collection.collectionState = Collection.CollectionState.INTAKE;

        if (gp1.isFirstRightBumper())
            brainSTEMRobot.collection.flickerState = Collection.FlickerState.UP_DOWN;

        if (gp1.isFirstY())
            if (brainSTEMRobot.shooter.shooterState == Shooter.ShooterState.UPDATE)
                brainSTEMRobot.shooter.shooterState = Shooter.ShooterState.OFF;
            else
                brainSTEMRobot.shooter.shooterState = Shooter.ShooterState.UPDATE;

        if (gp1.isFirstX())
            if (brainSTEMRobot.turret.turretState == Turret.TurretState.TRACKING)
                brainSTEMRobot.turret.turretState = Turret.TurretState.CENTER;
            else
                brainSTEMRobot.turret.turretState = Turret.TurretState.TRACKING;

//
        if (gp1.isFirstDpadUp()) {
            if (brainSTEMRobot.collection.collectionState == Collection.CollectionState.EXTAKE)
                brainSTEMRobot.collection.collectionState = Collection.CollectionState.OFF;
            else
                brainSTEMRobot.collection.collectionState = Collection.CollectionState.EXTAKE;
        }

        if (gp1.isFirstDpadLeft()) {
            brainSTEMRobot.turret.turretState = Turret.TurretState.PARK;
        }

        if (gp1.isFirstDpadRight()) {
            brainSTEMRobot.parking.parkState = Parking.ParkState.EXTENDED;
        }
    }

    private void updateDriver2() {
        if (gp2.isFirstB())
            if (brainSTEMRobot.collection.clutchState == Collection.ClutchState.ENGAGED)
                brainSTEMRobot.collection.clutchState = Collection.ClutchState.UNENGAGED;
            else
                brainSTEMRobot.collection.clutchState = Collection.ClutchState.ENGAGED;

        if (gp2.isFirstX())
            brainSTEMRobot.turret.isRedAlliance = false;
//        if (gp2.isFirstA())
//            brainSTEMRobot.turret.isRedAlliance = true;

        if (gp2.isFirstDpadLeft())
            brainSTEMRobot.turret.adjustment += 10;
        if (gp2.isFirstDpadRight())
            brainSTEMRobot.turret.adjustment -= 10;

        if (gp2.isFirstDpadUp())
            brainSTEMRobot.shooter.adjustment += 10;
        if (gp2.isFirstDpadDown())
            brainSTEMRobot.shooter.adjustment -= 10;

        if (gp2.isFirstRightBumper()) {
            if (brainSTEMRobot.turret.isRedAlliance) {
                PoseStorage.currentPose = new Pose2d(64, -65.5, Math.toRadians(180));
                brainSTEMRobot.drive.localizer.setPose(new Pose2d(64, -65.5, Math.toRadians(180)));
            } else {
                PoseStorage.currentPose = new Pose2d(64, 65.5, Math.toRadians(180));
                brainSTEMRobot.drive.localizer.setPose(new Pose2d(64, 65.5, Math.toRadians(180)));
            }
        }

        if (gp2.isFirstY()) {
            if (brainSTEMRobot.parking.parkState == Parking.ParkState.EXTENDED)
                brainSTEMRobot.turret.turretState = Turret.TurretState.PARK;
            else
                brainSTEMRobot.parking.parkState = Parking.ParkState.EXTENDED;
        }
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
        PinpointLocalizer pinpoint = brainSTEMRobot.drive.pinpoint();
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
    private void updatePosePredict() {
        // show predicted pose on dashboard
        PinpointLocalizer pinpoint = brainSTEMRobot.drive.pinpoint();
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
}