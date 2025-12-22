package com.example.motionProfiling;

public class TrapezoidalMotionProfiler {
    public static class Params {
        public double maxAccelInchesPerSec = 10;      // a_max (positive)
        public double maxVelocityInchesPerSec = 40;   // v_max (positive)
        public double maxDecelInchesPerSec = 10;      // d_max (positive)
    }
    public static Params params = new Params();

    public final double maxAccel, maxVel, maxDecel, totalError;

    public TrapezoidalMotionProfiler(double maxAccel, double maxVel, double maxDecel, double totalError) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.maxDecel = maxDecel;
        this.totalError = totalError;
    }
    public TrapezoidalMotionProfiler(double totalError) {
        this(params.maxAccelInchesPerSec, params.maxVelocityInchesPerSec, params.maxDecelInchesPerSec, totalError);
    }

    public DriveInfo calculate(double currentError) {
        double aMax = Math.abs(maxAccel);
        double vMax = Math.abs(maxVel);
        double dMax = Math.abs(maxDecel);

        // Direction is based on the original target direction
        double dir = (totalError >= 0) ? 1.0 : -1.0;

        double total = Math.abs(totalError);
        double remaining = Math.abs(currentError);

        // If we're basically there
        if (total <= 1e-9 || remaining <= 1e-6)
            return new DriveInfo(0, 0);

        // Progress along the path (0 at start, total at end)
        double progress = total - remaining;
        if (progress < 0) progress = 0;
        if (progress > total) progress = total;

        // Distances needed to accel to vMax, then decel from vMax
        double accelDistToVmax = (vMax * vMax) / (2.0 * aMax);
        double decelDistFromVmax = (vMax * vMax) / (2.0 * dMax);

        boolean isTrapezoid = (accelDistToVmax + decelDistFromVmax) <= total;

        double velMag;
        double accelMag;

        if (isTrapezoid) {
            double cruiseDist = total - accelDistToVmax - decelDistFromVmax;
            double decelStart = accelDistToVmax + cruiseDist;

            if (progress < accelDistToVmax) {
                // Accel phase: v = sqrt(2 a x)
                velMag = Math.sqrt(2.0 * aMax * progress);
                accelMag = aMax;
            } else if (progress < decelStart) {
                // Cruise phase
                velMag = vMax;
                accelMag = 0.0;
            } else {
                // Decel phase: v = sqrt(2 d * remaining)
                velMag = Math.sqrt(2.0 * dMax * remaining);
                accelMag = -dMax;
            }
        } else {
            // Triangle profile: peak velocity is below vMax
            // vPeak^2 = (2 * total * a * d) / (a + d)
            double vPeak = Math.sqrt((2.0 * total * aMax * dMax) / (aMax + dMax));

            double accelDist = (vPeak * vPeak) / (2.0 * aMax); // boundary between accel/decel

            if (progress < accelDist) {
                velMag = Math.sqrt(2.0 * aMax * progress);
                accelMag = aMax;
            } else {
                velMag = Math.sqrt(2.0 * dMax * remaining);
                accelMag = -dMax;
            }

            // Clamp just in case of numeric edge cases
            if (velMag > vPeak) velMag = vPeak;
        }

        // Final safety clamps
        if (velMag > vMax) velMag = vMax;

        double vel = dir * velMag;
        double accel = dir * accelMag;

        return new DriveInfo(vel, accel);
    }
}
