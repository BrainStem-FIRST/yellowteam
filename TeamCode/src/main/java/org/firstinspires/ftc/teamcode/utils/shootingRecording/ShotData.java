package org.firstinspires.ftc.teamcode.utils.shootingRecording;

public class ShotData {
    public final double timestamp, avgVel, targetVel, motorPower, ballExitAngleDeg;
    public ShotData(double timestamp, double avgVel, double targetVel, double motorPower, double ballExitAngleDeg) {
        this.timestamp = timestamp;
        this.avgVel = avgVel;
        this.targetVel = targetVel;
        this.motorPower = motorPower;
        this.ballExitAngleDeg = ballExitAngleDeg;
    }
}
