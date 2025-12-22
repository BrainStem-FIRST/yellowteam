package org.firstinspires.ftc.teamcode.utils.pidFeedForwardDrive;

import androidx.annotation.NonNull;

public class DriveInfo {
    public final double vel, accel;
    public DriveInfo(double vel, double accel) {
        this.vel = vel;
        this.accel = accel;
    }

    @NonNull
    @Override
    public String toString() {
        return vel + " " + accel;
    }
}
