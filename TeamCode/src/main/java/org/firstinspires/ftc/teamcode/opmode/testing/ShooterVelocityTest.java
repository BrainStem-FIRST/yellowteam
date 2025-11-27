package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name="Shooter Velocity Test", group="Data Recording")
@Config
public class ShooterVelocityTest extends OpMode {
    public static double startingVelocityTicksPerSec = 1100;
    public static double velocityIncrementAmount = 50;
    private boolean shooterOn;
    private Shooter shooter;
    private int curShotRecordIndex;
    private ElapsedTime recordTimer;
    @Override
    public void init() {
        shooter = new Shooter(hardwareMap, telemetry, null);
        shooterOn = false;
        recordTimer = new ElapsedTime();
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            shooterOn = !shooterOn;
            if (!shooterOn) {
                shooter.setShooterPower(0);
                curShotRecordIndex = 0;
                ShooterSpeedRecorder.incrementCurrentShot();
            }
        }

        if (shooterOn && ShooterSpeedRecorder.getCurrentShot() < ShooterSpeedRecorder.numShotsToRecord) {
            shooter.setShooterVelocityPID(startingVelocityTicksPerSec + ShooterSpeedRecorder.getCurrentShot() * velocityIncrementAmount);

            if (curShotRecordIndex >= ShooterSpeedRecorder.recordAmountForEachShot) {
                curShotRecordIndex = 0;
                ShooterSpeedRecorder.incrementCurrentShot();
            }

            if (recordTimer.milliseconds() > ShooterSpeedRecorder.recordIntervalMs) {
                recordTimer.reset();

                ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][curShotRecordIndex][0] = recordTimer.seconds();
                ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][curShotRecordIndex][1] = shooter.shooterPID.getTarget();
                ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][curShotRecordIndex][2] = shooter.getAvgMotorVelocity();
                ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][curShotRecordIndex][3] = shooter.shooterMotorHigh.getPower();
                curShotRecordIndex++;
            }
        }

        telemetry.addData("gamepad y", "toggle shooter");
        telemetry.addData("current shot", ShooterSpeedRecorder.getCurrentShot());
        telemetry.addData("target speed", shooter.shooterPID.getTarget());
        telemetry.update();
    }
}
