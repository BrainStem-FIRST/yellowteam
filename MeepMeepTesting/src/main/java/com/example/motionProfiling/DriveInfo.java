package com.example.motionProfiling;

public class DriveInfo {
    public final double vel, accel;
    public DriveInfo(double vel, double accel) {
        this.vel = vel;
        this.accel = accel;
    }

    @Override
    public String toString() {
        return vel + " " + accel;
    }
}
