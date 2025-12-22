package org.firstinspires.ftc.teamcode.utils.pidDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

@Config
public class PathParams {
    private static final double noMaxTime = -1;

    public static class DefaultParams {
        public double speedKp = 0.03, speedKi = 0, speedKd = 0.003, speedKpErrorPower = 2;
        public double headingKp = 0.01, headingKi = 0, headingKd = 0.00001, headingKdErrorPower = 2;
        public double applyKdLinearError = 10, applyKdHeadingRadError = 10;
        public double lateralWeight = 1.5, axialWeight = 1; // weight the drive powers to correct for differences in driving
        public double minSpeed = 0.18, maxSpeed = 1;
        public double minHeadingSpeed = 0.2, maxHeadingSpeed = 1;
        public double maxTime = 100;
    }
    public static DefaultParams defaultParams = new DefaultParams();
    public double lateralWeight, axialWeight;
    public double minLinearPower, maxLinearPower;
    public double minHeadingPower, maxHeadingPower;
    // decides how much the robot will slow down at this waypoint
    // if this equals 1, then the drivetrain will completely stop at this waypoint
    // if this equals 0, this waypoint will have no influence on slowing down the drivetrain as it approaches this point
    public double slowDownPercent;
    // if passPosition is true, the robot only needs to pass its target position for drive path to consider it "in tolerance", not fall within tolerance of it
    public boolean passPosition;
    public double maxTime;
    public BooleanSupplier customEndCondition = () -> false;

    public double speedKp, speedKi, speedKd, headingKp, headingKi, headingKd;
    public double speedKdErrorPower, headingKdErrorPower;
    public double applyKdLinearError, applyKdHeadingDegError;
    public PathParams() {
        this(defaultParams.speedKp, defaultParams.speedKi, defaultParams.speedKd, defaultParams.headingKp, defaultParams.headingKi, defaultParams.headingKd);
    }
    public PathParams(double[] pidCoefficients) {
        if (pidCoefficients.length != 6)
            throw new IllegalArgumentException("must pass in 8 PID coefficients. only passed in " + pidCoefficients.length + ": " + Arrays.toString(pidCoefficients));
        this.speedKp = pidCoefficients[0];
        this.speedKi = pidCoefficients[1];
        this.speedKd = pidCoefficients[2];
        this.headingKp = pidCoefficients[3];
        this.headingKi = pidCoefficients[4];
        this.headingKd = pidCoefficients[5];

        initializeDefault();
    }
    public PathParams(double speedKp, double speedKi, double speedKd, double headingKp, double headingKi, double headingKd) {
        this.speedKp = speedKp;
        this.speedKi = speedKi;
        this.speedKd = speedKd;
        this.headingKp = headingKp;
        this.headingKi = headingKi;
        this.headingKd = headingKd;

        initializeDefault();
    }
    private void initializeDefault() {
        maxTime = defaultParams.maxTime;
        minLinearPower = defaultParams.minSpeed;
        maxLinearPower = defaultParams.maxSpeed;
        minHeadingPower = defaultParams.minHeadingSpeed;
        maxHeadingPower = defaultParams.maxHeadingSpeed;
        lateralWeight = defaultParams.lateralWeight;
        axialWeight = defaultParams.axialWeight;
        passPosition = false;
        applyKdLinearError = defaultParams.applyKdLinearError;
        applyKdHeadingDegError = defaultParams.applyKdHeadingRadError;
        speedKdErrorPower = defaultParams.speedKpErrorPower;
        headingKdErrorPower = defaultParams.headingKdErrorPower;
    }
    public boolean hasMaxTime() {
        return maxTime != noMaxTime;
    }

    @NonNull
    @Override
    public PathParams clone() {
        PathParams newParams = new PathParams(speedKp, speedKi, speedKd, headingKp, headingKi, headingKd);
        newParams.maxTime = maxTime;
        newParams.minLinearPower = minLinearPower;
        newParams.maxLinearPower = maxLinearPower;
        newParams.minHeadingPower = minHeadingPower;
        newParams.maxHeadingPower = maxHeadingPower;
        newParams.lateralWeight = lateralWeight;
        newParams.axialWeight = axialWeight;
        newParams.passPosition = passPosition;
        newParams.customEndCondition = customEndCondition;
        newParams.applyKdLinearError = applyKdLinearError;
        newParams.applyKdHeadingDegError = applyKdHeadingDegError;
        newParams.speedKdErrorPower = speedKdErrorPower;
        newParams.headingKdErrorPower = headingKdErrorPower;
        return newParams;
    }
}
