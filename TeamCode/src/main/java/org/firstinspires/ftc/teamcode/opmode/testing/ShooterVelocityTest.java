package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.shootingRecording.ManualShooterSpeedRecorder;

@TeleOp(name="Shooter Velocity Test", group="Testing")
@Config
public class ShooterVelocityTest extends OpMode {
    public static double startingVelocityTicksPerSec = 1250;
    public static double velocityIncrementAmount = 50;
    private boolean shooterOn;
    private Shooter shooter;
    private int velNum;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        shooter = new Shooter(hardwareMap, telemetry, null);
        shooterOn = false;
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            shooterOn = !shooterOn;
            if (!shooterOn) {
                shooter.setShooterPower(0);
                velNum = 0;
            }
        }
        if (gamepad1.bWasPressed())
            velNum++;
        else if (gamepad1.xWasPressed())
            velNum--;

        if (shooterOn) {
            double targetVel = startingVelocityTicksPerSec + velNum * velocityIncrementAmount;
            shooter.setShooterVelocityPID(targetVel, shooter.getAvgMotorVelocity());
        }

        telemetry.addData("gamepad y", "toggle shooter");
        telemetry.addData("vel num", velNum);
        telemetry.addData("target speed", shooter.shooterPID.getTarget());
        telemetry.addData("current speed", shooter.getAvgMotorVelocity());
        telemetry.update();
    }
}
