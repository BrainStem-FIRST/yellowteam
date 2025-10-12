package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.commandGroups.FullCollectionSequence;
import org.firstinspires.ftc.teamcode.commands.CollectionCommand;
import org.firstinspires.ftc.teamcode.commands.TurretTrackingCommand;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.StickyButton;

@TeleOp(name = "TeleOp", group = "Robot")
public class BrainSTEMTeleOp extends LinearOpMode {
    BrainSTEMRobot brainSTEMRobot;

    // COMMANDS //
    private FullCollectionSequence collectionCommandGroup;
    private TurretTrackingCommand turretTrackingCommand;

    // STICKY AND TOGGLE BUTTONS //
    StickyButton gamepad1Left_Bumper = new StickyButton();
    StickyButton gamepad1Right_Bumper = new StickyButton();
    StickyButton gamepad1LeftTrigger = new StickyButton();
    StickyButton gamepad1RightTrigger = new StickyButton();
    StickyButton gamepad1AButton = new StickyButton();
    StickyButton gamepad1BButton = new StickyButton();
    StickyButton gamepad1XButton = new StickyButton();
    StickyButton gamepad1YButton = new StickyButton();
    StickyButton gamepad1DPADUPButton = new StickyButton();
    StickyButton gamepad1DPADDOWNButton = new StickyButton();
    StickyButton gamepad1DPADLEFTButton = new StickyButton();
    StickyButton gamepad1DPADRIGHTButton = new StickyButton();

    StickyButton gamepad2Left_Bumper = new StickyButton();
    StickyButton gamepad2Right_Bumper = new StickyButton();
    StickyButton gamepad2LeftTrigger = new StickyButton();
    StickyButton gamepad2RightTrigger = new StickyButton();
    StickyButton gamepad2AButton = new StickyButton();
    StickyButton gamepad2BButton = new StickyButton();
    StickyButton gamepad2XButton = new StickyButton();
    StickyButton gamepad2YButton = new StickyButton();
    StickyButton gamepad2DPADUPButton = new StickyButton();
    StickyButton gamepad2DPADDOWNButton = new StickyButton();
    StickyButton gamepad2DPADLEFTButton = new StickyButton();
    StickyButton gamepad2DPADRIGHTButton = new StickyButton();

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(20);

        brainSTEMRobot = new BrainSTEMRobot(telemetry, hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));

        collectionCommandGroup = new FullCollectionSequence(brainSTEMRobot, telemetry);
        turretTrackingCommand = new TurretTrackingCommand(brainSTEMRobot, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            updateButton();
            updateDrive();
            updateDriver1();
            updateDriver2();
            CommandScheduler.getInstance().run();
            brainSTEMRobot.update();
            brainSTEMRobot.drive.updatePoseEstimate();
            telemetry.update();
        }
    }
     private void updateButton(){
        gamepad1Left_Bumper.update(gamepad1.left_bumper);
        gamepad1Right_Bumper.update(gamepad1.right_bumper);
        gamepad1LeftTrigger.update(gamepad1.left_trigger > 0.2);
        gamepad1RightTrigger.update(gamepad1.right_trigger > 0.2);
        gamepad1AButton.update(gamepad1.a);
        gamepad1BButton.update(gamepad1.b);
        gamepad1XButton.update(gamepad1.x);
        gamepad1YButton.update(gamepad1.y);
        gamepad1DPADUPButton.update(gamepad1.dpad_up);
        gamepad1DPADDOWNButton.update(gamepad1.dpad_down);
        gamepad1DPADRIGHTButton.update(gamepad1.dpad_right);
        gamepad1DPADLEFTButton.update(gamepad1.dpad_left);

        gamepad2Left_Bumper.update(gamepad2.left_bumper);
        gamepad2Right_Bumper.update(gamepad2.right_bumper);
        gamepad2LeftTrigger.update(gamepad2.left_trigger > 0.2);
        gamepad2RightTrigger.update(gamepad2.right_trigger > 0.2);
        gamepad2AButton.update(gamepad2.a);
        gamepad2BButton.update(gamepad2.b);
        gamepad2XButton.update(gamepad2.x);
        gamepad2YButton.update(gamepad2.y);
        gamepad2DPADUPButton.update(gamepad2.dpad_up);
        gamepad2DPADDOWNButton.update(gamepad2.dpad_down);
        gamepad2DPADRIGHTButton.update(gamepad2.dpad_right);
        gamepad2DPADLEFTButton.update(gamepad2.dpad_left);
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

    private void updateDriver1() {
        if (gamepad1AButton.getState())
            collectionCommandGroup.schedule();

        if (gamepad1BButton.getState())
            brainSTEMRobot.collection.collectionState = Collection.CollectionState.EXTAKE;
        else
            brainSTEMRobot.collection.collectionState = Collection.CollectionState.OFF;

        if (gamepad1YButton.getState())
            brainSTEMRobot.shooter.shooterMotor.setPower(0.89 * Shooter.SHOOTER_PARAMS.TORQUE_CONSTANT);
        else
            brainSTEMRobot.shooter.shooterMotor.setPower(0);

        if (gamepad1XButton.getState())
            turretTrackingCommand.schedule();
    }

    private void updateDriver2() {
        if (gamepad2BButton.getState())
            brainSTEMRobot.turret.isRedAlliance = true;
        if (gamepad2XButton.getState())
            brainSTEMRobot.turret.isRedAlliance = false;
    }
}