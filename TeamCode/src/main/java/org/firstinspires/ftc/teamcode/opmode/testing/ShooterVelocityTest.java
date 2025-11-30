package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.shootingRecording.ManualShooterSpeedRecorder;

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
                ManualShooterSpeedRecorder.incrementCurrentShot();
            }
        }

        if (shooterOn && ManualShooterSpeedRecorder.getCurrentShot() < ManualShooterSpeedRecorder.numShotsToRecord) {
            shooter.setShooterVelocityPID(startingVelocityTicksPerSec + ManualShooterSpeedRecorder.getCurrentShot() * velocityIncrementAmount);

            if (curShotRecordIndex >= ManualShooterSpeedRecorder.recordAmountForEachShot) {
                curShotRecordIndex = 0;
                ManualShooterSpeedRecorder.incrementCurrentShot();
            }

            if (recordTimer.milliseconds() > ManualShooterSpeedRecorder.recordIntervalMs) {
                recordTimer.reset();

                ManualShooterSpeedRecorder.data[ManualShooterSpeedRecorder.getCurrentShot()][curShotRecordIndex][0] = recordTimer.seconds();
                ManualShooterSpeedRecorder.data[ManualShooterSpeedRecorder.getCurrentShot()][curShotRecordIndex][1] = shooter.shooterPID.getTarget();
                ManualShooterSpeedRecorder.data[ManualShooterSpeedRecorder.getCurrentShot()][curShotRecordIndex][2] = shooter.getAvgMotorVelocity();
                ManualShooterSpeedRecorder.data[ManualShooterSpeedRecorder.getCurrentShot()][curShotRecordIndex][3] = shooter.shooterMotorHigh.getPower();
                curShotRecordIndex++;
            }
        }

        telemetry.addData("gamepad y", "toggle shooter");
        telemetry.addData("current shot", ManualShooterSpeedRecorder.getCurrentShot());
        telemetry.addData("target speed", shooter.shooterPID.getTarget());
        telemetry.update();
    }
}
