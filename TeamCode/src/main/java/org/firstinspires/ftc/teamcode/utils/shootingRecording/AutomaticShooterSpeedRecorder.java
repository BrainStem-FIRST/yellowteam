package org.firstinspires.ftc.teamcode.utils.shootingRecording;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMath;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;

import java.util.ArrayList;

@Config
@TeleOp(name="Automatic Shooter Speed Recorder", group="Data Recording")
public class AutomaticShooterSpeedRecorder extends OpMode {
    public static int bigScrollAmount = 4;
    public static int shotBufferFrames = 10;
    public static int sameShotThresholdFrames = 2; // how far apart do 2 decelerations need to be for this to consider them as separate shots?
    private static final ArrayList<ShotData> rawData = new ArrayList<>();
    private static final ArrayList<ArrayList<ShotData>> data = new ArrayList<>();

    public static void resetData() {
        rawData.clear();
        data.clear();
    }
    public static void addShotData(ShotData shot) {
        rawData.add(shot);
    }
    private static void parseRawData() {
        data.clear();
        ArrayList<Integer> shotInts = new ArrayList<>();
        double prevVel = rawData.get(0).avgVel;
        double curVel;
        for (int i=1; i<rawData.size(); i++) {
            curVel = rawData.get(i).avgVel;

            if (curVel - prevVel <= Shooter.SHOOTER_PARAMS.minShootBallDeceleration) {
                boolean farEnoughFromPrev = shotInts.isEmpty() || i - shotInts.get(shotInts.size() - 1) > sameShotThresholdFrames;
                if (farEnoughFromPrev)
                    shotInts.add(i);
            }
        }
        for (Integer shotInt : shotInts) {
            data.add(new ArrayList<>());

            for (int i=Math.max(0, shotInt - shotBufferFrames); i<Math.min(rawData.size(), shotInt + shotBufferFrames); i++) {
                data.get(data.size() - 1).add(rawData.get(i));
            }
        }
    }
    private GamepadTracker g1;
    private int currentShownShot = 0;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        g1 = new GamepadTracker(gamepad1);
    }

    @Override
    public void start() {
        updateTelemetry();
    }

    @Override
    public void loop() {
        int oldCurrentShownShot = currentShownShot;
        g1.update();

        if (gamepad1.y)
            resetData();
        if (g1.isFirstA())
            parseRawData();

        if (!data.isEmpty()) {
            if (g1.isFirstDpadUp())
                currentShownShot++;
            else if (g1.isFirstDpadDown())
                currentShownShot--;
            else if (g1.isFirstDpadRight())
                currentShownShot += bigScrollAmount;
            else if (g1.isFirstDpadLeft())
                currentShownShot -= bigScrollAmount;

            currentShownShot = (currentShownShot + data.size()) % data.size(); // wrap around
        }

        boolean newShot = currentShownShot != oldCurrentShownShot;
        if (newShot)
            updateTelemetry();

    }
    private void updateTelemetry() {
        telemetry.addLine("===CONTROLS===");
        telemetry.addData("parse raw data", "A");
        telemetry.addData("reset speeds", "Y");
        telemetry.addData("scroll through recorded shots", "dpad up/down");
        telemetry.addData("big scroll through recorded shots", "dpad right/left");
        telemetry.addLine();
        telemetry.addLine("===HYPER PARAMS===");
        telemetry.addData("current shot index", currentShownShot);
        telemetry.addData("num shots recorded", data.size());
        telemetry.addLine();
        telemetry.addLine("===DATA (time, target spd, actual spd, power, hood) ===");
        ArrayList<ShotData> shot = data.get(currentShownShot);
        for (ShotData shotData : shot) {
            String outOfBoundsAngle = shotData.ballExitAngleDeg > ShootingMath.hoodSystemParams.maxAngleDeg || shotData.ballExitAngleDeg < ShootingMath.hoodSystemParams.minAngleDeg ?
                    ", OUT OF HOOD RANGE" : "";
            telemetry.addLine("time: " + MathUtils.format3(shotData.timestamp) +
                    ", targetSpd: " + MathUtils.format2(shotData.targetVel) +
                    ", actualSpd: " + MathUtils.format2(shotData.avgVel) +
                    ", power: " + MathUtils.format3(shotData.motorPower) +
                    ", exitAngle: " + MathUtils.format3(shotData.ballExitAngleDeg) +
                    outOfBoundsAngle
            );
        }
        telemetry.update();
    }
}
