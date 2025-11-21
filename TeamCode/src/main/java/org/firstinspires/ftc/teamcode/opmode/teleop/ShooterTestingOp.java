package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;

@TeleOp(name="SHOOTER TEST", group="Competition")
public class ShooterTestingOp extends LinearOpMode {

    BrainSTEMRobot robot;
    GamepadTracker gp1;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(20);
        CommandScheduler.getInstance().reset();

        robot = new BrainSTEMRobot(Alliance.BLUE, telemetry, hardwareMap, PoseStorage.currentPose);
        gp1 = new GamepadTracker(gamepad1);

        waitForStart();
        while (opModeIsActive()) {
            robot.shooter.setHoodPosition(Shooter.testingHoodPosition);
            if (Shooter.useVelocity)
                robot.shooter.setShooterVelocityPID(Shooter.testingShootVelocity);
            else
                robot.shooter.setShooterPower(Shooter.testingShootPower);


            telemetry.addData("SHOOTER HIGH POWER", robot.shooter.shooterMotorHigh.getPower());
            telemetry.addData("SHOOTER LOW POWER", robot.shooter.shooterMotorHigh.getPower());
            telemetry.addData("SHOOTER HIGH VELOCITY", robot.shooter.shooterMotorHigh.getVelocity());
            telemetry.addData("SHOOTER LOW VELOCITY", robot.shooter.shooterMotorLow.getVelocity());

            telemetry.update();
        }
    }
}
