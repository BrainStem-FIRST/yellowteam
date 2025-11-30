package org.firstinspires.ftc.teamcode.utils.pidDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

@Config
public class PathParams {
    private static final double noMaxTime = -1;

    public static class DefaultParams {
        public double speedKp = 0.05, speedKi = 0.001, speedKd = 0.004;
        public double headingKp = 0.6, headingKi = 0.01, headingKd = 0;
        public double lateralWeight = 1.2, axialWeight = 1; // weight the drive powers to correct for differences in driving
        public double minSpeed = 0.2, maxSpeed = 1;
        public double minHeadingSpeed = 0, maxHeadingSpeed = Double.MAX_VALUE;
        public double maxTime = 2;
    }
    public static DefaultParams defaultParams = new DefaultParams();
    public double lateralWeight, axialWeight;
    public double minSpeed, maxSpeed;
    public double minHeadingSpeed, maxHeadingSpeed;
    // decides how much the robot will slow down at this waypoint
    // if this equals 1, then the drivetrain will completely stop at this waypoint
    // if this equals 0, this waypoint will have no influence on slowing down the drivetrain as it approaches this point
    public double slowDownPercent;
    // if passPosition is true, the robot only needs to pass its target position for drive path to consider it "in tolerance", not fall within tolerance of it
    public boolean passPosition;
    public double maxTime;
    public BooleanSupplier customEndCondition = () -> false;

    public double speedKp, speedKi, speedKd, headingKp, headingKi, headingKd;

    public PathParams() {
        this(defaultParams.speedKp, defaultParams.speedKi, defaultParams.speedKd, defaultParams.headingKp, defaultParams.headingKi, defaultParams.headingKd);
    }
    public PathParams(double[] pidCoefficients) {
        if (pidCoefficients.length != 6)
            throw new IllegalArgumentException("must pass in 6 PID coefficients. only passed in " + pidCoefficients.length + ": " + Arrays.toString(pidCoefficients));
        this.speedKp = pidCoefficients[0];
        this.speedKi = pidCoefficients[1];
        this.speedKd = pidCoefficients[2];
        this.headingKp = pidCoefficients[3];
        this.headingKi = pidCoefficients[4];
        this.headingKd = pidCoefficients[5];

        maxTime = defaultParams.maxTime;
        minSpeed = defaultParams.minSpeed;
        maxSpeed = defaultParams.maxSpeed;
        minHeadingSpeed = defaultParams.minHeadingSpeed;
        maxHeadingSpeed = defaultParams.maxHeadingSpeed;
        lateralWeight = defaultParams.lateralWeight;
        axialWeight = defaultParams.axialWeight;
        passPosition = false;
    }
    public PathParams(double speedKp, double speedKi, double speedKd, double headingKp, double headingKi, double headingKd) {
        this.speedKp = speedKp;
        this.speedKi = speedKi;
        this.speedKd = speedKd;
        this.headingKp = headingKp;
        this.headingKi = headingKi;
        this.headingKd = headingKd;

        maxTime = defaultParams.maxTime;
        minSpeed = defaultParams.minSpeed;
        maxSpeed = defaultParams.maxSpeed;
        minHeadingSpeed = defaultParams.minHeadingSpeed;
        maxHeadingSpeed = defaultParams.maxHeadingSpeed;
        lateralWeight = defaultParams.lateralWeight;
        axialWeight = defaultParams.axialWeight;
        passPosition = false;
    }
    public boolean hasMaxTime() {
        return maxTime != noMaxTime;
    }

    @NonNull
    @Override
    public PathParams clone() {
        PathParams newParams = new PathParams(speedKp, speedKi, speedKd, headingKp, headingKi, headingKd);
        newParams.maxTime = maxTime;
        newParams.minSpeed = minSpeed;
        newParams.maxSpeed = maxSpeed;
        newParams.minHeadingSpeed = minHeadingSpeed;
        newParams.maxHeadingSpeed = maxHeadingSpeed;
        newParams.lateralWeight = lateralWeight;
        newParams.axialWeight = axialWeight;
        newParams.passPosition = passPosition;
        newParams.customEndCondition = customEndCondition;
        return newParams;
    }
}
