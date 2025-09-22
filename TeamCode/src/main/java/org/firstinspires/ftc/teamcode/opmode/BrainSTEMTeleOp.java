package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.MathFunctions;

import java.net.ProxySelector;

@TeleOp(name = "TeleOp Test", group = "Robot")
public class BrainSTEMTeleOp extends LinearOpMode {
    Pose2d targetPose = new Pose2d(72, 72, 0);
    BrainSTEMRobot brainSTEMRobot;

    @Override
    public void runOpMode() {

        MathFunctions mathFunctions = new MathFunctions();
        brainSTEMRobot = new BrainSTEMRobot(telemetry, hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));

        waitForStart();

        while (opModeIsActive()) {

            brainSTEMRobot.update();
            Pose2d currentPose = brainSTEMRobot.drive.localizer.getPose();

            // drivetrain turning
            double turnPower = mathFunctions.getTurnPower(currentPose, targetPose);

            brainSTEMRobot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x //turnPower
            ));

            if (gamepad1.a)
                brainSTEMRobot.turret.setTurretPosition(150);

            if (gamepad1.y)
                brainSTEMRobot.turret.pointTurretAtTarget(currentPose, targetPose);

//            if (gamepad1.b)
//                brainSTEMRobot.shooter.shooterMotor.setPower(-1);
//            else
//                brainSTEMRobot.shooter.shooterMotor.setPower(0);


//            telemetry.addData("Pose X", currentPose.position.x);
//            telemetry.addData("Pose Y", currentPose.position.y);
//            telemetry.addData("Pose Heading", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.update();
        }
    }
}
