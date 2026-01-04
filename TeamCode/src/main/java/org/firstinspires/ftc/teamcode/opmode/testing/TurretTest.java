package org.firstinspires.ftc.teamcode.opmode.testing;

import static org.firstinspires.ftc.teamcode.subsystems.Turret.turretParams;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name="Turret Test", group="Testing")
@Config
public class TurretTest extends OpMode {
    public static double hoodPosition = 0.3;
    private Turret turret;
    private Shooter shooter;
    @Override
    public void init() {
        turret = new Turret(hardwareMap, telemetry, null);
        turret.resetEncoders();

        shooter = new Shooter(hardwareMap, telemetry, null);
    }

    @Override
    public void loop() {
        shooter.setHoodPosition(hoodPosition);

        double turretAngleDeg = turret.getTurretEncoder() * 1.0 / turretParams.TICKS_PER_REV * 360;
        telemetry.addData("turret encoder", turret.getTurretEncoder());
        telemetry.addData("turret angle deg", turretAngleDeg);
        telemetry.update();
    }
}
