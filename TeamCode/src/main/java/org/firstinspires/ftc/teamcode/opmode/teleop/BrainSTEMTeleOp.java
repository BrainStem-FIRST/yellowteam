package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.commandGroups.FullCollectionSequence;
import org.firstinspires.ftc.teamcode.commands.turretCommands.TurretTrackingCommand;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Parking;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;

@TeleOp(name = "TeleOp", group = "Robot")
public class BrainSTEMTeleOp extends LinearOpMode {
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

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(20); // faster telemetry speed

        brainSTEMRobot = new BrainSTEMRobot(telemetry, hardwareMap, PoseStorage.currentPose); //take pose from auto
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);

        collectionCommandGroup = new FullCollectionSequence(brainSTEMRobot, telemetry);
        turretTrackingCommand = new TurretTrackingCommand(brainSTEMRobot, telemetry);
        CommandScheduler.getInstance().reset();

        waitForStart();

        while (opModeIsActive()) {
            gp1.update();
            gp2.update();
            updateDrive();
//            updateDriver1();
            updateDriver2();
            updateTesting();
            CommandScheduler.getInstance().run();
            brainSTEMRobot.update();
            brainSTEMRobot.drive.updatePoseEstimate();
            telemetry.update();
        }

        brainSTEMRobot.vision.visionPortal.close();
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
//                brainSTEMRobot.collection.flickerState = Collection.FlickerState.DOWN;
//            else
//                brainSTEMRobot.collection.flickerState = Collection.FlickerState.UP_DOWN;
        }

//        if (gp1.isFirstDpadDown())
//            if (brainSTEMRobot.shooter.shooterState == Shooter.ShooterState.UPDATE_2)
//                brainSTEMRobot.shooter.shooterState = Shooter.ShooterState.OFF;
//            else
//                brainSTEMRobot.shooter.shooterState = Shooter.ShooterState.UPDATE_2;
//
//        if (gp1.isFirstDpadRight()) {
//            if (parking_position + Parking.PARK_PARAMS.SERVO_INCREMENT <= 1.0)
//                parking_position += Parking.PARK_PARAMS.SERVO_INCREMENT;
//        }

//        if (gp1.isFirstDpadUp()) {
//            if (hood_position - Shooter.SHOOTER_PARAMS.HOOD_INCREMENT >= 0.05)
//                hood_position -= Shooter.SHOOTER_PARAMS.HOOD_INCREMENT;
//        }
//
//        if (gp1.isFirstDpadDown()) {
//            if (hood_position + Shooter.SHOOTER_PARAMS.HOOD_INCREMENT <= 0.95)
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

        telemetry.addData("FLICKER POS", brainSTEMRobot.collection.flickerRight.getPosition());

        telemetry.addData("Pose X", brainSTEMRobot.drive.localizer.getPose().position.x);
        telemetry.addData("Pose Y", brainSTEMRobot.drive.localizer.getPose().position.y);
        telemetry.addData("Pose Heading", Math.toDegrees(brainSTEMRobot.drive.localizer.getPose().heading.toDouble()));
    }

    private void updateDriver1() {
        if (gp1.isFirstA())
            if (brainSTEMRobot.collection.collectionState == Collection.CollectionState.INTAKE)
                brainSTEMRobot.collection.collectionState = Collection.CollectionState.OFF;
            else
                brainSTEMRobot.collection.collectionState = Collection.CollectionState.INTAKE;

        if (gp1.isFirstB())
            if (brainSTEMRobot.collection.flickerState == Collection.FlickerState.UP)
                brainSTEMRobot.collection.flickerState = Collection.FlickerState.DOWN;
            else
                brainSTEMRobot.collection.flickerState = Collection.FlickerState.UP;

        if (gp1.isFirstY())
            if (brainSTEMRobot.shooter.shooterState == Shooter.ShooterState.UPDATE)
                brainSTEMRobot.shooter.shooterState = Shooter.ShooterState.OFF;
            else
                brainSTEMRobot.shooter.shooterState = Shooter.ShooterState.UPDATE;

        if (gp1.isFirstX())
            if (brainSTEMRobot.turret.turretState == Turret.TurretState.TRACKING)
                brainSTEMRobot.turret.turretState = Turret.TurretState.OFF;
            else
                brainSTEMRobot.turret.turretState = Turret.TurretState.TRACKING;

//
        if (gp1.isFirstDpadUp()) {
            if (brainSTEMRobot.collection.collectionState == Collection.CollectionState.EXTAKE)
                brainSTEMRobot.collection.collectionState = Collection.CollectionState.OFF;
            else
                brainSTEMRobot.collection.collectionState = Collection.CollectionState.EXTAKE;
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
        if (gp2.isFirstA())
            brainSTEMRobot.turret.isRedAlliance = true;
        if (gp2.isFirstDpadLeft())
            brainSTEMRobot.turret.adjustment += 10;
        if (gp2.isFirstDpadRight())
            brainSTEMRobot.turret.adjustment -= 10;
    }
}