package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.MathFunctions;

import java.net.ProxySelector;

@TeleOp(name = "TeleOp Test", group = "Robot")
public class BrainSTEMTeleOp extends LinearOpMode {
    Pose2d targetPose = new Pose2d(72, -72, 0);

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        MathFunctions mathFunctions = new MathFunctions();

        waitForStart();

        while (opModeIsActive()) {

            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();
            double turnPower = mathFunctions.getTurnPower(currentPose, targetPose);

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    turnPower // -gamepad1.right_stick_x
            ));

            telemetry.addData("Pose X", currentPose.position.x);
            telemetry.addData("Pose Y", currentPose.position.y);
            telemetry.addData("Pose Heading", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }
    }
}
