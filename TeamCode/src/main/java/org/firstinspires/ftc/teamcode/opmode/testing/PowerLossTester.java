package org.firstinspires.ftc.teamcode.opmode.testing;

import static org.firstinspires.ftc.teamcode.utils.math.MathUtils.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMath;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

@TeleOp(name="Power Loss Tester", group="Testing")
@Config
public class PowerLossTester extends OpMode {
    public static class Controls {
        public double targetShooterVelocityTicksPerSec = 0;
        public double ballExitAngleRad = 0;
        public boolean powerShooter = true;
    }
    public static class Experiment {
        public double gravityAcceleration = 9.81;
        public double[] startPose = { 0, 0, 0 };

        // represents distances the the balls landed from the center of the robot
        public double[] distanceInches = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
    }
    public static Controls controls = new Controls();
    public static Experiment experiment = new Experiment();

    private BrainSTEMRobot robot;

    double avgDistMeters, changeInYMeters;
    double shooterVelTicksPerSec, shooterVelMetersPerSec;
    double[] theoreticalDistMeters;
    double actualExitVelocityMetersPerSecUsingRealWorldData;
    double powerLossCoefficient;
    double actualExitVelocityMetersPerSecUsingPowerLoss;
    double[] expectedDistancesOfTravel;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, createPose(experiment.startPose));

        robot.collection.clutchRight.setPosition(Collection.params.ENGAGED_POS);
        robot.collection.clutchLeft.setPosition(Collection.params.ENGAGED_POS);
    }
    @Override
    public void loop() {
        if (gamepad1.aWasPressed())
            controls.powerShooter = !controls.powerShooter;

        double collectPower = controls.powerShooter ? Collection.params.INTAKE_SPEED : 0;
        robot.collection.collectorMotor.setPower(collectPower);

        if (controls.targetShooterVelocityTicksPerSec == 0 || !controls.powerShooter)
            robot.shooter.setShooterPower(0);
        else {
            shooterVelTicksPerSec = robot.shooter.getAvgMotorVelocity();
            robot.shooter.setShooterVelocityPID(controls.targetShooterVelocityTicksPerSec, shooterVelTicksPerSec);
            robot.shooter.setHoodPosition(ShootingMath.calculateHoodServoPosition(controls.ballExitAngleRad));

            avgDistMeters = calculateAvgDist(experiment.distanceInches) * 0.0254;
            if (avgDistMeters < 0)
                telemetry.addLine("__INPUT DISTANCES ARE NOT VALID");

            // change in y = final y - initial y
            changeInYMeters = ShootingMath.shooterSystemParams.ballRadiusMeters - ShootingMath.calculateExactExitHeightMeters(controls.ballExitAngleRad);

            shooterVelMetersPerSec = ShootingMath.ticksPerSecToExitSpeedMps(shooterVelTicksPerSec, 1);

            theoreticalDistMeters = calculateExpectedDistanceOfTravel(changeInYMeters, controls.ballExitAngleRad, shooterVelMetersPerSec);

            actualExitVelocityMetersPerSecUsingRealWorldData = calculateActualExitVelocity(avgDistMeters, changeInYMeters, controls.ballExitAngleRad);
            if (actualExitVelocityMetersPerSecUsingRealWorldData < 0) {
                telemetry.addLine("actual exit velocity using real world data is not valid");
                telemetry.update();
                return;
            }

            powerLossCoefficient = actualExitVelocityMetersPerSecUsingRealWorldData / shooterVelMetersPerSec;
            actualExitVelocityMetersPerSecUsingPowerLoss = ShootingMath.ticksPerSecToExitSpeedMps(shooterVelTicksPerSec, powerLossCoefficient); // this should equal actualExitVelocityMetersPerSecUsingRealWorldData
            expectedDistancesOfTravel = calculateExpectedDistanceOfTravel(changeInYMeters, controls.ballExitAngleRad, actualExitVelocityMetersPerSecUsingPowerLoss); // this should equal avgDistMeters

            telemetry.addLine("MISC=====");
            telemetry.addData("left hood pos", robot.shooter.hoodLeftServo.getPosition());
            telemetry.addLine();
            telemetry.addLine("CALCULATIONS============");
            telemetry.addData("change in Y from ball exit position (meters)", changeInYMeters);
            telemetry.addData("average recorded landing distance from ball exit position (meters)", MathUtils.format3(avgDistMeters));
            telemetry.addData("average theoretical landing distance without power loss (meters)", MathUtils.format3(theoreticalDistMeters));
            telemetry.addLine();
            telemetry.addData("shooter angular velocity ticks per sec", MathUtils.format3(shooterVelTicksPerSec));
            telemetry.addData("shooter tangential velocity meters per sec", MathUtils.format3(shooterVelMetersPerSec));
            telemetry.addData("actual ball exit velocity using real world data (meters per sec)", MathUtils.format3(actualExitVelocityMetersPerSecUsingRealWorldData));
            telemetry.addData("actual ball exit velocity using power loss (meters per sec)", MathUtils.format3(actualExitVelocityMetersPerSecUsingPowerLoss));
            telemetry.addLine();
            telemetry.addData("expected distance of travel 1 with power loss (meters)", MathUtils.format3(expectedDistancesOfTravel[0]));
            telemetry.addData("expected distance of travel 2 with power loss (meters)", MathUtils.format3(expectedDistancesOfTravel[1]));
            telemetry.addLine();
            telemetry.addData("calculated power loss coefficient", MathUtils.format(powerLossCoefficient, 8));

            telemetry.update();
        }
    }

    private double calculateAvgDist(double[] distances) {
        // under the assumption that the turret is facing the robot's direction, only the x offset of the exit position matters
        Vector2d exitPosition = ShootingMath.calculateExitPositionInches(createPose(experiment.startPose), 0, controls.ballExitAngleRad);
        double xOffset = exitPosition.x - BrainSTEMRobot.length / 2.; // should set up tape measurer at the front of robot (where intake is)

        double total = 0;
        int numDists = 0;
        for (double dist : distances) {
            if (dist < 0)
                continue;
            numDists++;

            double actualDist = dist - xOffset;
            total += actualDist;
        }
        if (numDists == 0)
            return -1;
        return total / numDists;
    }

    private double calculateActualExitVelocity(double distanceMeters, double changeInYMeters, double exitAngleRad) {
        double cosTheta = Math.cos(exitAngleRad);
        double tanTheta = Math.tan(exitAngleRad);
        double denominator = 2 * cosTheta * cosTheta * (distanceMeters * tanTheta - changeInYMeters);
        if (denominator <= 0)
            return -1;
        double numerator = experiment.gravityAcceleration * distanceMeters * distanceMeters;
        return Math.sqrt(numerator / denominator);
    }
    private double[] calculateExpectedDistanceOfTravel(double changeInYMeters, double exitAngleRad, double exitVelMetersPerSec) {
        double g = experiment.gravityAcceleration;
        double y = changeInYMeters;
        double v = exitVelMetersPerSec;
        double cosTheta = Math.cos(exitAngleRad);
        double tanTheta = Math.tan(exitAngleRad);

        double discriminantSquared = tanTheta * tanTheta - 2 * g * y / (v * v * cosTheta * cosTheta);
        if (discriminantSquared < 0)
            return new double[] { -1, -1 };

        double discriminant = Math.sqrt(discriminantSquared);
        double term1 = v * v * cosTheta * cosTheta / g;

        double solution1 = term1 * (tanTheta + discriminant);
        double solution2 = term1 * (tanTheta - discriminant);
        return new double[] { solution1, solution2 };
    }
}
