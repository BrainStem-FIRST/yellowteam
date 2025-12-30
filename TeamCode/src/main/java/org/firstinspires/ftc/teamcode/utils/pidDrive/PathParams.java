package org.firstinspires.ftc.teamcode.utils.pidDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import java.util.function.BooleanSupplier;

@Config
public class PathParams {
    public enum HeadingLerpType {
        LINEAR,
        TANGENT
    }
    private static final double noMaxTime = -1;

    public static class DefaultParams {
        public double closeSpeedKp = 0.02, closeSpeedKi = 0, closeSpeedKd = 0.01, speedKf = 0.115;
        public double farSpeedKp = 0.035, farSpeedKi = 0, farSpeedKd = 0;
        public double applyCloseSpeedPIDError = 5;
        public double closeHeadingKp = 0.005, closeHeadingKi = 0, closeHeadingKd = 0.00005, headingKf = 0.17;
        public double farHeadingKp = 0.015, farHeadingKi = 0, farHeadingKd = 0;
        public double applyCloseHeadingPIDErrorDeg = 10;
        public double applyKdLinearError = 10;
        public double lateralWeight = 1.9, axialWeight = 1; // weight the drive powers to correct for differences in driving
        public double minSpeed = 0, maxSpeed = 1;
        public double minHeadingSpeed = 0, maxHeadingSpeed = 1;
        public double maxTime = 100;
        public HeadingLerpType headingLerpType = HeadingLerpType.LINEAR;
        public double tangentHeadingActivateThreshold = 10;
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

    public double closeSpeedKp, closeSpeedKi, closeSpeedKd, speedKf;
    public double farSpeedKp, farSpeedKi, farSpeedKd;
    public double applyCloseSpeedPIDError;
    public double closeHeadingKp, closeHeadingKi, closeHeadingKd, farHeadingKp, farHeadingKi, farHeadingKd, headingKf;
    public double applyKdLinearError;
    public HeadingLerpType headingLerpType;
    public double tangentHeadingActivateThreshold, applyCloseHeadingPIDErrorDeg;
    public PathParams() {
        this(defaultParams.closeSpeedKp, defaultParams.closeSpeedKi, defaultParams.closeSpeedKd, defaultParams.farSpeedKp, defaultParams.farSpeedKi, defaultParams.farSpeedKd, defaultParams.speedKf, defaultParams.closeHeadingKp, defaultParams.closeHeadingKi, defaultParams.closeHeadingKd, defaultParams.farHeadingKp, defaultParams.farHeadingKi, defaultParams.farHeadingKd, defaultParams.headingKf);
    }
    public PathParams(double closeSpeedKp, double closeSpeedKi, double closeSpeedKd, double farSpeedKp, double farSpeedKi, double farSpeedKd, double speedKf, double closeHeadingKp, double closeHeadingKi, double closeHeadingKd, double farHeadingKp, double farHeadingKi, double farHeadingKd, double headingKf) {
        this.closeSpeedKp = closeSpeedKp;
        this.closeSpeedKi = closeSpeedKi;
        this.closeSpeedKd = closeSpeedKd;
        this.farSpeedKp = farSpeedKp;
        this.farSpeedKi = farSpeedKi;
        this.farSpeedKd = farSpeedKd;
        this.speedKf = speedKf;

        this.closeHeadingKp = closeHeadingKp;
        this.closeHeadingKi = closeHeadingKi;
        this.closeHeadingKd = closeHeadingKd;
        this.farHeadingKp = farHeadingKp;
        this.farHeadingKi = farHeadingKi;
        this.farHeadingKd = farHeadingKd;
        this.headingKf = headingKf;

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
        applyCloseSpeedPIDError = defaultParams.applyCloseSpeedPIDError;
        headingLerpType = defaultParams.headingLerpType;
        tangentHeadingActivateThreshold = defaultParams.tangentHeadingActivateThreshold;
        applyCloseHeadingPIDErrorDeg = defaultParams.applyCloseHeadingPIDErrorDeg;
    }
    public boolean hasMaxTime() {
        return maxTime != noMaxTime;
    }

    @NonNull
    @Override
    public PathParams clone() {
        PathParams newParams = new PathParams(closeSpeedKp, closeSpeedKi, closeSpeedKd, farSpeedKp, farSpeedKi, farSpeedKd, speedKf, closeHeadingKp, closeHeadingKi, closeHeadingKd, farHeadingKp, farHeadingKi, farHeadingKd, headingKf);
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
        newParams.headingLerpType = headingLerpType;
        newParams.tangentHeadingActivateThreshold = tangentHeadingActivateThreshold;
        newParams.applyCloseHeadingPIDErrorDeg = applyCloseHeadingPIDErrorDeg;
        return newParams;
    }
}
