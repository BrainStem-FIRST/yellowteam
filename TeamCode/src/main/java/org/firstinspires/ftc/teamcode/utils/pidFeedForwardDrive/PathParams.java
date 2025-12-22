package org.firstinspires.ftc.teamcode.utils.pidFeedForwardDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import java.util.function.BooleanSupplier;

@Config
public class PathParams {
    private static final double noMaxTime = -1;
    public static class PIDParams {
        public double speedKp = 0, speedKi = 0, speedKd = 0;
        public double headingKp = 0, headingKi = 0, headingKd = 0;
    }
    public static class FeedforwardParams {
        public double speedKs = 0, speedKv = 0, speedKa = 0;
        public double headingKs = 0, headingKv = 0, headingKa = 0;
    }
    public static class MiscParams {
        public double axialWeight = 1, lateralWeight = 1;
        public double minSpeed = 0.0, maxSpeed = 1;
        public double minHeadingSpeed = 0, maxHeadingSpeed = 1;
        public double maxTime = 100;
    }
    public static PIDParams pid = new PIDParams();
    public static FeedforwardParams ff = new FeedforwardParams();
    public static MiscParams misc = new MiscParams();
    public TrapezoidalMotionProfile translationalMotionProfile, headingRadMotionProfile;
    public double axialWeight, lateralWeight;
    public double minLinearVoltage, maxLinearVoltage;
    public double minHeadingPower, maxHeadingPower;
    // decides how much the robot will slow down at this waypoint
    // if this equals 1, then the drivetrain will completely stop at this waypoint
    // if this equals 0, this waypoint will have no influence on slowing down the drivetrain as it approaches this point
    public double slowDownPercent;
    // if passPosition is true, the robot only needs to pass its target position for drive path to consider it "in tolerance", not fall within tolerance of it
    public boolean passPosition;
    public double maxTime;
    public BooleanSupplier customEndCondition = () -> false;

    public PathParams() {
        translationalMotionProfile = new TrapezoidalMotionProfile(TrapezoidalMotionProfile.params.translational);
        headingRadMotionProfile = new TrapezoidalMotionProfile(TrapezoidalMotionProfile.params.headingRad);
        axialWeight = misc.axialWeight;
        lateralWeight = misc.lateralWeight;
        maxTime = misc.maxTime;
        minLinearVoltage = misc.minSpeed;
        maxLinearVoltage = misc.maxSpeed;
        minHeadingPower = misc.minHeadingSpeed;
        maxHeadingPower = misc.maxHeadingSpeed;
        passPosition = false;
    }
    public boolean hasMaxTime() {
        return maxTime != noMaxTime;
    }

    @NonNull
    @Override
    public PathParams clone() {
        PathParams newParams = new PathParams();
        newParams.translationalMotionProfile = translationalMotionProfile;
        newParams.headingRadMotionProfile = headingRadMotionProfile;
        newParams.axialWeight = axialWeight;
        newParams.lateralWeight = lateralWeight;
        newParams.maxTime = maxTime;
        newParams.minLinearVoltage = minLinearVoltage;
        newParams.maxLinearVoltage = maxLinearVoltage;
        newParams.minHeadingPower = minHeadingPower;
        newParams.maxHeadingPower = maxHeadingPower;
        newParams.passPosition = passPosition;
        newParams.customEndCondition = customEndCondition;
        return newParams;
    }
}
