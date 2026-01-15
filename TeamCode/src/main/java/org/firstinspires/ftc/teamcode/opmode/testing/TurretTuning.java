package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(group="Testing", name="turret tuner")
public class TurretTuning extends LinearOpMode {
    public static double powerCoeff;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        waitForStart();
        while(opModeIsActive()) {
            turretMotor.setPower(powerCoeff);
            telemetry.addData("power", powerCoeff);
            telemetry.addData("encoder", turretMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
