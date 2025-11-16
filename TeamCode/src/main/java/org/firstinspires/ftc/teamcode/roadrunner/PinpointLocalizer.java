package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.utils.OdoInfo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class PosePredictParams {
        public double startingDtEstimation = 0.02;
        public int numPrevDeltaTimesToTrack = 10;
        public double[] dtWeights = {0.3, 0.2, 0.15, 0.125, 0.075, 0.05, 0.03, 0.025, 0.025, 0.02 }; // MUST add up to 1

        public int numPrevVelocitiesToTrack = 4;
        public double maxLinearSpeedPerSecond = 10, maxHeadingDegSpeedPerSecond = 20;
        public boolean clampSpeeds = false;
        public double[] accelerationWeights = { 0.6, 0.3, 0.1 }; // MUST add up to 1
        public double velocityDamping = 0.75;
    }
    public static class Params {
        public double parYTicks = -0.946; // y position of the parallel encoder (in tick units)
        public double perpXTicks = -7.328; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();
    public static PosePredictParams posePredictParams = new PosePredictParams();


    public final GoBildaPinpointDriver driver;
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    // previousVelocities[0] = most recent
    // previousAccelerations[0] is most recent, calculated with previousVelocities[0] and previousVelocities[1]
    public final ArrayList<Double> previousDeltaTimes;
    public final ArrayList<OdoInfo> previousVelocities, previousAccelerations;
    private long lastUpdateTimeNano;
    public Pose2d lastPose;
    private int framesRunning;
    public PinpointLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        double mmPerTick = inPerTick * 25.4;
        driver.setEncoderResolution(20, DistanceUnit.MM); //1.0 / mmPerTick (FIX VALUE)
        driver.setOffsets(
                DistanceUnit.MM.fromInches(PARAMS.parYTicks),
                DistanceUnit.MM.fromInches(PARAMS.perpXTicks),
                DistanceUnit.MM
        );

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        driver.setEncoderDirections(initialParDirection, initialPerpDirection);

        driver.resetPosAndIMU();

        txWorldPinpoint = initialPose;
        lastPose = initialPose;

        framesRunning = 0;
        lastUpdateTimeNano = 0;
        previousDeltaTimes = new ArrayList<>(); // stores recent delta times (seconds)
        previousVelocities = new ArrayList<>();
        previousAccelerations = new ArrayList<>();

        if (posePredictParams.accelerationWeights.length != posePredictParams.numPrevVelocitiesToTrack - 1)
            throw new IllegalArgumentException(("PosePredictParams has " + posePredictParams.accelerationWeights.length + " accelerationWeights but also specifies " + posePredictParams.numPrevVelocitiesToTrack + " velocities to track. These do not match"));
        double sumOfAccelWeights = Arrays.stream(posePredictParams.accelerationWeights).sum();
        if (Math.abs(sumOfAccelWeights - 1) > 0.00001)
            throw new IllegalArgumentException(("the acceleration weights in PosePredictParams do not add up to 1. They add up to " + sumOfAccelWeights));
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }


    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            framesRunning++;
            lastPose = getPose();

            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            double velX = driver.getVelX(DistanceUnit.INCH), velY = driver.getVelY(DistanceUnit.INCH), velHeadingRad = driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            Vector2d worldVelocity = new Vector2d(velX, velY);
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            updatePreviousVelocitiesAndAccelerations(velX, velY, velHeadingRad);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    private void updatePreviousVelocitiesAndAccelerations(double vx, double vy, double vh) {
        double dt = (System.nanoTime() - lastUpdateTimeNano) * 1.0 * 1e-9; // delta time is in seconds
        if (previousDeltaTimes.size() >= posePredictParams.numPrevDeltaTimesToTrack)
            previousDeltaTimes.remove(previousDeltaTimes.size() - 1);
        previousDeltaTimes.add(0, dt);

        // remove oldest velocity
        if (previousVelocities.size() >= posePredictParams.numPrevVelocitiesToTrack)
            previousVelocities.remove(previousVelocities.size() - 1);

        // add most recent velocity (and clamp if necessary)
        if (posePredictParams.clampSpeeds) {
            double linearSpeedPerSecond = Math.sqrt(vx * vx + vy * vy);
            if (linearSpeedPerSecond > posePredictParams.maxLinearSpeedPerSecond) {
                vx *= posePredictParams.maxLinearSpeedPerSecond / linearSpeedPerSecond;
                vy *= posePredictParams.maxLinearSpeedPerSecond / linearSpeedPerSecond;
            }
            if (Math.abs(vh) > Math.toRadians(posePredictParams.maxHeadingDegSpeedPerSecond))
                vh = Math.signum(vh) * Math.toRadians(posePredictParams.maxHeadingDegSpeedPerSecond);
        }
        previousVelocities.add(0, new OdoInfo(vx, vy, vh));

        // remove oldest acceleration
        if (previousAccelerations.size() >= posePredictParams.numPrevVelocitiesToTrack - 1)
            previousAccelerations.remove(previousAccelerations.size() - 1);

        // update acceleration
        // acceleration = current velocity - old velocity
        // not dividing by time b/c velocity is already in the desired "time" unit - change from last frame to this frame
        // so this acceleration actually represents the change in velocity from last frame to this frame
        if (previousVelocities.size() > 1)
            previousAccelerations.add(0, new OdoInfo(
                    (previousVelocities.get(0).x - previousVelocities.get(1).x) / dt,
                    (previousVelocities.get(0).y - previousVelocities.get(1).y) / dt,
                    (previousVelocities.get(0).headingRad - previousVelocities.get(1).headingRad) / dt
            ));
        lastUpdateTimeNano = System.nanoTime();
    }

    public boolean canGetNextPoseSimple() {
        return !previousVelocities.isEmpty();
    }
    public boolean canGetNextPoseAdvanced() {
        return !previousAccelerations.isEmpty();
    }
    // predicts on most recent velocity
    public Pose2d getNextPoseSimple(double time) {
        if (previousVelocities.isEmpty())
            return getPose();
        double velocityMultiplier = time * posePredictParams.velocityDamping;
        Pose2d pose = getPose();
        return new Pose2d(
                pose.position.x + previousVelocities.get(0).x * velocityMultiplier,
                pose.position.y + previousVelocities.get(0).y * velocityMultiplier,
                pose.heading.toDouble() + previousVelocities.get(0).headingRad * velocityMultiplier
        );
    }
    // predicts based on acceleration
    public Pose2d getNextPoseAdvanced() {
        if (previousAccelerations.isEmpty())
            return getPose();
        // find weighted average of acceleration
        OdoInfo weightedAcceleration = new OdoInfo();
        for (int i=0; i<previousAccelerations.size(); i++) {
            weightedAcceleration.x += previousAccelerations.get(i).x * posePredictParams.accelerationWeights[i];
            weightedAcceleration.y += previousAccelerations.get(i).y * posePredictParams.accelerationWeights[i];
            weightedAcceleration.headingRad += previousAccelerations.get(i).headingRad * posePredictParams.accelerationWeights[i];
        }

        double predictedDt = getWeightedDt();
        // predict the velocity of next frame
        // new velocity = old velocity + acceleration
        OdoInfo nextVelocity = previousVelocities.get(0);
        nextVelocity.x += weightedAcceleration.x * predictedDt;
        nextVelocity.y += weightedAcceleration.y * predictedDt;
        nextVelocity.headingRad += weightedAcceleration.headingRad * predictedDt;

        // get velocity into correct time unit
        nextVelocity.x *= predictedDt * posePredictParams.velocityDamping;
        nextVelocity.y *= predictedDt * posePredictParams.velocityDamping;
        nextVelocity.headingRad *= predictedDt * posePredictParams.velocityDamping;

        // predict the position of next frame
        // new position = old position + velocity
        Pose2d pose = getPose();
        return new Pose2d(pose.position.x + nextVelocity.x, pose.position.y + nextVelocity.y, pose.heading.toDouble() + nextVelocity.headingRad);
    }
    public double getWeightedDt() {
        if (framesRunning < posePredictParams.numPrevDeltaTimesToTrack)
            return posePredictParams.startingDtEstimation;

        double dt = 0;
        for (int i=0; i<previousDeltaTimes.size(); i++)
            dt += previousDeltaTimes.get(i) * posePredictParams.dtWeights[i];
        return dt;
    }
}
