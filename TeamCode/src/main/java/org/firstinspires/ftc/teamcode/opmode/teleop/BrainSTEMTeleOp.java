package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.MathFunctions;
import org.firstinspires.ftc.teamcode.utils.StickyButton;

@TeleOp(name = "TeleOp Test", group = "Robot")
public class BrainSTEMTeleOp extends LinearOpMode {

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

    Pose2d targetPose = new Pose2d(72, 72, 0);
    BrainSTEMRobot brainSTEMRobot;

    @Override
    public void runOpMode() {

        MathFunctions mathFunctions = new MathFunctions();
        brainSTEMRobot = new BrainSTEMRobot(telemetry, hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));

        waitForStart();

        while (opModeIsActive()) {

//            updateButton();
            updateDrive(brainSTEMRobot);
//            updateDriver1(brainSTEMRobot);
//            updateDriver2(brainSTEMRobot);
            CommandScheduler.getInstance().run();
            brainSTEMRobot.update();
            brainSTEMRobot.drive.updatePoseEstimate();

            Pose2d currentPose = brainSTEMRobot.drive.localizer.getPose();
            double turnPower = mathFunctions.getTurnPower(currentPose, targetPose);
        }
    }

//            if (gamepad1.a)
//                brainSTEMRobot.turret.setTurretPosition(150);
//
//            if (gamepad1.y)
//                brainSTEMRobot.turret.turretState = Turret.TurretState.TRACKING;
//
//            if (gamepad1.b)
//                brainSTEMRobot.shooter.shooterMotor.setPower(0.89 * Shooter.SHOOTER_PARAMS.TORQUE_CONSTANT);
//            else
//                brainSTEMRobot.shooter.shooterMotor.setPower(0);
//
//
//        // BUTTON CONTROLS //
//
//        private void updateButton(){
//            gamepad1Y.update(gamepad1.y);
//            feildCentric.update(gamepad1.left_stick_button);
//            gamepad1Left_Bumper.update(gamepad1.left_bumper);
//            gamepad1Right_Bumper.update(gamepad1.right_bumper);
//            gamepad1LeftStickButton.update(gamepad1.left_stick_button);
//            gamepad1RightStickButton.update(gamepad1.right_stick_button);
//            gamepad2DpadLeft.update(gamepad2.dpad_left);
//            gamepad2DpadRight.update(gamepad2.dpad_right);
//            gamepad1AButton.update(gamepad1.a);
//            gamepad1BButton.update(gamepad1.b);
//            gamepad1DPADUPButton.update(gamepad1.dpad_up);
//            gamepad1DPADDOWNButton.update(gamepad1.dpad_down);
//            gamepad2DPADUPButton.update(gamepad2.dpad_up);
//            gamepad2DPADDOWNButton.update(gamepad2.dpad_down);
//            gamepad1DPADRIGHTButton.update(gamepad1.dpad_right);
//            gamepad1DPADLEFTButton.update(gamepad1.dpad_left);
//            gamepad2A.update(gamepad2.a);
//            gamepad2B.update(gamepad2.b);
//            gamepad2X.update(gamepad2.x);
//            gamepad2Y.update(gamepad2.y);
//            gamepad2RightStickButton.update(gamepad2.right_stick_button);
//            gamepad2LeftStickButton.update(gamepad2.left_stick_button);
//            gamepad2RightTrigger.update(gamepad2.right_trigger > 0.2);
//            gamepad2LeftBumper.update(gamepad2.left_bumper);
//            gamepad2RightBumper.update(gamepad2.right_bumper);
//            gamepad1LeftTrigger.update(gamepad1.left_trigger > 0.2);
//            gamepad1RightTrigger.update(gamepad1.right_trigger > 0.2);
//            gamepad1X.update(gamepad1.x);
//            gamepad2LeftRightStickButton.update(gamepad2.left_stick_button && gamepad2.right_stick_button);
//        }
//
//
//        private void updateDriver1(BrainSTEMRobot robot) {
//            driver1LiftControls(robot);
//            driver1CollectionPickupControl(robot);
//            driver1ResetControls(robot);
//            //driver1ExtensionOutControl(robot);
////        driver1ExtensionControls(robot);
////        driver1LiftControls(robot);
////        driver1IntakeControls(robot);
////        driver1TroughControls(robot);
//        }
//
//        private void updateDriver2(BrainSTEMRobot robot) {
//            driver2CollectionPickupSetStateControls(robot);
//            driver2DepositSetStateControls(robot);
//            driver2HangingControls(robot);
//            driver2CollectionControls(robot);
////        driver2IntakeControls(robot);
////        driver2ExtensionControls(robot);
////        driver2FliparmControls(robot);
////        driver2EndEffectorControls(robot);
////        driver2HangingControls(robot);
//        }



        // DT CONTROLS //

        private void updateDrive(BrainSTEMRobot robot) {
            brainSTEMRobot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x //turnPower
            ));
        }




        // DRIVER 1 CONTROLS //



//        private void driver1CollectionPickupControl(BrainSTEMRobot robot) {
//            if(hangingStates == HangingStates.NOT_HANGING) {
//                if (gamepad1Left_Bumper.getState() && collectionStates == CollectionStates.SUBMERSIBLE_PICKUP) {
//                    new ExtensionOutCommand(robot.collectorExtension, telemetry).schedule();
//                } else if (gamepad1Left_Bumper.getState() && collectionStates == CollectionStates.WALL_PICKUP) {
//                    new PreCollectionSequenceWallCommandGroup(robot, telemetry).schedule();
//                } else if (gamepad1.left_trigger >= 0.2 && (collectionStates == CollectionStates.SUBMERSIBLE_PICKUP || (robot.collectorExtension.getExtensionState() == CollectorExtension.ExtensionState.OUT || robot.collectorExtension.getExtensionState() == CollectorExtension.ExtensionState.INCREMENT_OUT || robot.collectorExtension.getExtensionState() == CollectorExtension.ExtensionState.INCREMENT_IN) )) {
//                    new CollectionSequenceCommand(robot, telemetry).schedule();
//                } else if (gamepad1.left_trigger >= 0.2 && collectionStates == CollectionStates.WALL_PICKUP) {
//                    new CollectionSequenceWallCommandGroup(robot, telemetry).schedule();
//                } else if (gamepad1Y.getState()) {
//                    new WholeTransferSequenceCommand(robot, telemetry).schedule();
//                } else if (gamepad1X.getState()) {
//                    new TransferSequenceCommand(robot, telemetry).schedule();
//                }
//            }
//        }
//
//        private void driver1LiftControls(BrainSTEMRobot robot) {
//            if(hangingStates == HangingStates.NOT_HANGING) {
//                if (gamepad1AButton.getState()) {
//                    new DepositSequenceCommand(robot, telemetry).schedule();
//                }
//                else if (gamepad1Right_Bumper.getState() && robot.lift.depositStates == DepositStates.SPECIMEN_HIGH) {
//                    new RetractSequenceCommand(robot, telemetry, gamepad1.right_trigger).schedule();
////            Log.d("LIFT DEPOSIT ", "IF LP ");
//                    // new RetractSequenceCommand(robot, telemetry, gamepad1.right_trigger).schedule();
//                } else if (gamepad1RightTrigger.getState() && !(robot.lift.depositStates == DepositStates.SPECIMEN_HIGH)) {
//                    new BasketRetractSequenceCommand(robot, telemetry).schedule();
//                } else if (gamepad1RightTrigger.getState() && robot.lift.depositStates == DepositStates.SPECIMEN_HIGH) {
//                    new PreRetractSequenceCommandTele(robot, telemetry).schedule();
//                    //new ExtensionOutCommand(robot.collectorExtension, telemetry).schedule();
//                }
//            }
//
//        }
//        private void driver1ResetControls(BrainSTEMRobot robot) {
//            if(gamepad1BButton.getState()){
//                new AllResetCommand(robot, telemetry).schedule();
//            }
//        }

//        }
//    }
}
