package org.firstinspires.ftc.teamcode.utils.shootingRecording;

public class ShotData {
    public final double timestamp, avgVel, targetVel, motorPower, ballExitAngleDeg;
    public final int turretEncoder;
    public final double targetTurretEncoder;
    public ShotData(double timestamp, double avgVel, double targetVel, double motorPower, double ballExitAngleDeg, int turretEncoder, double targetTurretEncoder) {
        this.timestamp = timestamp;
        this.avgVel = avgVel;
        this.targetVel = targetVel;
        this.motorPower = motorPower;
        this.ballExitAngleDeg = ballExitAngleDeg;
        this.turretEncoder = turretEncoder;
        this.targetTurretEncoder = targetTurretEncoder;
    }
}
