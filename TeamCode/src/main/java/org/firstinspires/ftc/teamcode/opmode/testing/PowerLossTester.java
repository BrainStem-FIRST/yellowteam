package org.firstinspires.ftc.teamcode.opmode.testing;

import static org.firstinspires.ftc.teamcode.utils.math.MathUtils.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMath;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

@TeleOp(name="Power Loss Tester", group="Testing")
@Config
public class PowerLossTester extends OpMode {
    public static double gravityAcceleration = 9.81;
    public static boolean useHighArc = false;
    public static double exitAngleRad = 0;
    public static double targetShooterVelocityTicksPerSec = 0;
    public static double[] startPose = { 0, 0, 0 };
    public static double[] distanceInches = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
    private BrainSTEMRobot robot;


    double avgDistInches, avgDistMeters;
    double shooterVelTicksPerSec, shooterVelMetersPerSec;
    double[] theoreticalDistInches, theoreticalDistMeters;
    double actualExitVelocityMetersPerSecFromMath;
    double powerLossCoefficient;
    double actualExitVelocityMetersPerSec;
    double[] expectedDistancesOfTravel;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, createPose(startPose));
    }
    @Override
    public void loop() {
        if (targetShooterVelocityTicksPerSec != 0) {
            double currentShooterVelocityTicksPerSec = robot.shooter.getAvgMotorVelocity();
            robot.shooter.setShooterVelocityPID(targetShooterVelocityTicksPerSec, currentShooterVelocityTicksPerSec);
            robot.shooter.setHoodPosition(ShootingMath.calculateHoodServoPosition(exitAngleRad));

            double changeInYMeters = -ShootingMath.calculateExitHeightMeters(useHighArc);

            avgDistInches = calculateAvgDist(distanceInches);
            avgDistMeters = avgDistInches * 0.0254;

            shooterVelTicksPerSec = robot.shooter.getAvgMotorVelocity();
            shooterVelMetersPerSec = ShootingMath.ticksPerSecToExitSpeedMps(currentShooterVelocityTicksPerSec, 1);

            theoreticalDistMeters = calculateExpectedDistanceOfTravel(changeInYMeters, exitAngleRad, shooterVelMetersPerSec);
            theoreticalDistInches = new double[]{theoreticalDistMeters[0] / 0.0254, theoreticalDistMeters[1] / 0.0254};

            if (avgDistInches > 0 && shooterVelMetersPerSec > 0) {
                actualExitVelocityMetersPerSecFromMath = calculateActualExitVelocity(avgDistMeters, changeInYMeters, exitAngleRad);
                powerLossCoefficient = actualExitVelocityMetersPerSecFromMath / shooterVelMetersPerSec;
                actualExitVelocityMetersPerSec = ShootingMath.ticksPerSecToExitSpeedMps(robot.shooter.getAvgMotorVelocity(), powerLossCoefficient);
                expectedDistancesOfTravel = calculateExpectedDistanceOfTravel(changeInYMeters, exitAngleRad, actualExitVelocityMetersPerSec);
            }
        }

        telemetry.addLine("ROBOT DATA===============");
        telemetry.addData("shooter velocity ticks per sec", MathUtils.format3(shooterVelTicksPerSec));
        telemetry.addData("shooter tangential velocity meters per sec", MathUtils.format3(shooterVelMetersPerSec));
        telemetry.addData("actual ball exit velocity meters per sec", MathUtils.format3(actualExitVelocityMetersPerSec));
        telemetry.addData("expected distance of travel 1", MathUtils.format3(expectedDistancesOfTravel[0]));
        telemetry.addData("expected distance of travel 2", MathUtils.format3(expectedDistancesOfTravel[1]));

        telemetry.addLine("REAL WORLD DATA============");
        telemetry.addData("average recorded landing distance (inches)", MathUtils.format3(avgDistInches));
        telemetry.addData("average recorded landing distance (meters)", MathUtils.format3(avgDistMeters));

        telemetry.addLine("THEORETICAL DATA============");
        telemetry.addData("theoretical landing distance (inches)", MathUtils.format3(theoreticalDistInches));
        telemetry.addData("theoretical landing distance (meters)", MathUtils.format3(theoreticalDistMeters));
        telemetry.addLine();
        telemetry.addData("power loss coefficient", MathUtils.format(powerLossCoefficient, 8));

        telemetry.update();
    }

    private double calculateAvgDist(double[] distances) {
        double total = 0;
        int numDists = 0;
        for (double dist : distances) {
            if (dist < 0)
                continue;

            numDists++;
            total += dist;
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
        double numerator = gravityAcceleration * distanceMeters * distanceMeters;
        return Math.sqrt(numerator / denominator);
    }
    private double[] calculateExpectedDistanceOfTravel(double changeInYMeters, double exitAngleRad, double actualExitVelMetersPerSec) {
        double g = gravityAcceleration;
        double y = changeInYMeters;
        double v = actualExitVelMetersPerSec;
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
