package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.opmode.Alliance;

@TeleOp(name="Reset Turret Encoder", group="Competition")
public class ResetTurretEncoderTele extends OpMode {
    @Override
    public void init() {
        BrainSTEMRobot robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, new Pose2d(0, 0, 0));
        robot.turret.resetEncoders();
        telemetry.addLine("reset turret encoder");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addLine("reset turret encoder");
        telemetry.update();
    }

    @Override
    public void loop() {

    }
}
