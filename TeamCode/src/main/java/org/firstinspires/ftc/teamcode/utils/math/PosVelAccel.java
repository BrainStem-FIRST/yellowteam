package org.firstinspires.ftc.teamcode.utils.math;

public class PosVelAccel {
    private double pos;
    private double vel;
    private double accel;

    public void update(double newPos, double dt) {
        double prevPos = pos;
        pos = newPos;

        double prevVel = vel;
        vel = (pos - prevPos) / dt;

        accel = (vel - prevVel) / dt;
    }
    public void update(double newPos, double newVel, double dt) {
        pos = newPos;

        double prevVel = vel;
        vel = newVel;

        accel = (vel - prevVel) / dt;
    }
    public double getPos() {
        return pos;
    }
    public double getVel() {
        return vel;
    }
    public double getAccel() {
        return accel;
    }

}
