package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp (name = "TeleOp Test", group = "Robot")
public class BrainSTEMTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x));

            drive.updatePoseEstimate();

            telemetry.addData("Pose X", drive.localizer.getPose().position.x);
            telemetry.addData("Pose Y", drive.localizer.getPose().position.y);
            telemetry.addData("Pose Heading", drive.localizer.getPose().heading.toDouble());
            telemetry.update();
        }
    }
}