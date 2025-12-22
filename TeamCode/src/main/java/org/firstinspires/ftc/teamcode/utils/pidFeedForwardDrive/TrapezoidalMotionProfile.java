package org.firstinspires.ftc.teamcode.utils.pidFeedForwardDrive;

public class TrapezoidalMotionProfile {
    public static class Params {
        //  { acceleration, max speed, deceleration }
        public double[] translational = new double[] { 75, 100, 100 };
        public double[] headingRad = new double[] { Math.PI, Math.PI, Math.PI };
    }
    public static Params params = new Params();

    public final double maxAccel, maxVel, maxDecel;
    private double totalError;

    public TrapezoidalMotionProfile(double[] params) {
        this(params[0], params[1], params[2]);
    }
    public TrapezoidalMotionProfile(double maxAccel, double maxVel, double maxDecel) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.maxDecel = maxDecel;
        totalError = 0;
    }
    public void setTotalError(double totalError) {
        this.totalError = totalError;
    }

    public DriveInfo calculate(double currentError) {
        if (totalError == 0)
            return new DriveInfo(0, 0);

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
