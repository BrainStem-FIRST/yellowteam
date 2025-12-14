package org.firstinspires.ftc.teamcode.utils.shootingRecording;

public class ShotData {
    public final double timestamp, avgVel, theoreticalTargetVel, adjustedTargetVel, motorPower, ballExitAngleDeg;
    public final int numBallsShot, turretEncoder;
    public final double targetTurretEncoder;
    public ShotData(double timestamp, int numBallsShot, double avgVel, double theoreticalTargetVel, double adjustedTargetVel, double motorPower, double ballExitAngleDeg, int turretEncoder, double targetTurretEncoder) {
        this.timestamp = timestamp;
        this.numBallsShot = numBallsShot;
        this.avgVel = avgVel;
        this.theoreticalTargetVel = theoreticalTargetVel;
        this.adjustedTargetVel = adjustedTargetVel;
        this.motorPower = motorPower;
        this.ballExitAngleDeg = ballExitAngleDeg;
        this.turretEncoder = turretEncoder;
        this.targetTurretEncoder = targetTurretEncoder;
    }
}
