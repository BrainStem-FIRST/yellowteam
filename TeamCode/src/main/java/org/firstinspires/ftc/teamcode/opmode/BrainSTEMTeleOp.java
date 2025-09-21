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
    Pose2d targetPose = new Pose2d(72, -72, 0);
    BrainSTEMRobot brainSTEMRobot;
    private DcMotorEx shooterMotor;

    @Override
    public void runOpMode() {

        MathFunctions mathFunctions = new MathFunctions();
        brainSTEMRobot = new BrainSTEMRobot(telemetry, hardwareMap, new Pose2d(0, 0, 0));
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

            if (gamepad1.b)
                shooterMotor.setPower(-1);
            else
                shooterMotor.setPower(0);

//            telemetry.addData("Pose X", currentPose.position.x);
//            telemetry.addData("Pose Y", currentPose.position.y);
//            telemetry.addData("Pose Heading", Math.toDegrees(currentPose.heading.toDouble()));
//            telemetry.addData("Turn Power", turnPower);
            telemetry.addData("Shooter Encoder", shooterMotor.getCurrentPosition());
            telemetry.addData("Shooter Current", shooterMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
