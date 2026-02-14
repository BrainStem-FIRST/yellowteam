package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name="Reset Turret Encoder", group="Competition")
public class ResetTurretEncoderTele extends LinearOpMode {
    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap, telemetry);
        turret.resetEncoders();
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("turret encoder", turret.getEncoder());
            telemetry.update();
        }

    }
}
