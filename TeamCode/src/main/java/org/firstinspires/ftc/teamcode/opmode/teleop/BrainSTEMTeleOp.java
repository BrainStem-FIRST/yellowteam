package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.commandGroups.FullCollectionSequence;
import org.firstinspires.ftc.teamcode.commands.collectionCommands.CollectionCommand;
import org.firstinspires.ftc.teamcode.commands.turretCommands.TurretTrackingCommand;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.StickyButton;

@TeleOp(name = "TeleOp", group = "Robot")
public class BrainSTEMTeleOp extends LinearOpMode {
    BrainSTEMRobot brainSTEMRobot;

    // COMMANDS //
    private FullCollectionSequence collectionCommandGroup;
    private TurretTrackingCommand turretTrackingCommand;
    GamepadTracker gp1;
    GamepadTracker gp2;

    //testing
    private double hood_position = 0.9;

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
//            updateDriver2();
            updateTesting();
            CommandScheduler.getInstance().run();
            brainSTEMRobot.update();
            brainSTEMRobot.drive.updatePoseEstimate();
            telemetry.update();
        }
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
            if (brainSTEMRobot.shooter.shooterState == Shooter.ShooterState.SHOOT)
                brainSTEMRobot.shooter.shooterState = Shooter.ShooterState.OFF;
            else
                brainSTEMRobot.shooter.shooterState = Shooter.ShooterState.SHOOT;

        if (gp1.isFirstDpadUp()) {
            if (hood_position - 0.1 >= 0.1)
                hood_position -= 0.1;
        }

        if (gp1.isFirstDpadDown()) {
            if (hood_position + 0.1 <= 0.9)
                hood_position += 0.1;
        }

        brainSTEMRobot.shooter.hoodServo.setPosition(hood_position);
    }

    private void updateDriver1() {
        if (gp1.isFirstA())
            collectionCommandGroup.schedule();

        if (gamepad1.b)
            brainSTEMRobot.collection.collectionState = Collection.CollectionState.EXTAKE;
        else
            brainSTEMRobot.collection.collectionState = Collection.CollectionState.OFF;

        if (gp1.isFirstY()) {
            brainSTEMRobot.collection.clutchState = Collection.ClutchState.ENGAGED;
            brainSTEMRobot.collection.collectionState = Collection.CollectionState.TRANSFER;
        }

        if (gp1.isFirstX())
            turretTrackingCommand.schedule();
    }

    private void updateDriver2() {
        if (gp2.isFirstB())
            brainSTEMRobot.turret.isRedAlliance = true;
        if (gp2.isFirstX())
            brainSTEMRobot.turret.isRedAlliance = false;
    }
}