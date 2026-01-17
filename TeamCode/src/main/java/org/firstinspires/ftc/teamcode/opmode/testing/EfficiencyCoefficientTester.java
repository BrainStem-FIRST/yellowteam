package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMath;

@TeleOp(name="Power Efficiency Tester", group="Testing")
@Config
public class EfficiencyCoefficientTester extends OpMode {
    public static class Controls {
        public double ballExitAngleRad = Math.toRadians(40);
        public boolean powerShooter = true;
        public boolean powerIntake = true;
        public double efficiencyCoefficient = 0.62;
        public double distanceToShootBallInches = 150; // distance to shoot ball from back of robot
        public double heightToShootBallInches = 0; // height to shoot ball from ground
    }
    public static class Experiment {
        public double gravityAcceleration = 9.81;
    }
    public static Controls controls = new Controls();
    public static Experiment experiment = new Experiment();

    private BrainSTEMRobot robot;

    double totalDistanceTraveledMeters, changeInYMeters;
    double shooterVelTicksPerSec, shooterVelMetersPerSec;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, new Pose2d(0, 0, 0));

        robot.collection.clutchRight.setPosition(Collection.params.ENGAGED_POS);
        robot.collection.clutchLeft.setPosition(Collection.params.ENGAGED_POS);
    }
    @Override
    public void loop() {
        robot.shooter.setHoodPosition(ShootingMath.calculateHoodServoPosition(controls.ballExitAngleRad));

        if (gamepad1.aWasPressed())
            controls.powerShooter = !controls.powerShooter;
        if(gamepad1.rightBumperWasPressed())
            controls.powerIntake = !controls.powerIntake;

        double collectPower = controls.powerIntake ? Collection.params.INTAKE_SPEED : 0;
        robot.collection.collectorMotor.setPower(collectPower);

        if (!controls.powerShooter)
            robot.shooter.setShooterPower(0);
        else {
            shooterVelTicksPerSec = robot.shooter.getAvgMotorVelocity();
            shooterVelMetersPerSec = ShootingMath.ticksPerSecToExitSpeedMps(shooterVelTicksPerSec, 1);

            // under the assumption that the turret is facing the robot's direction, only the x offset of the exit position matters
            Pose2d start = new Pose2d(0, 0, 0);
            Vector2d exitPosition = ShootingMath.calculateExitPositionInches(start, 0, controls.ballExitAngleRad);
            double initialX = exitPosition.x + BrainSTEMRobot.length * 0.5;
            double finalX = controls.distanceToShootBallInches;
            totalDistanceTraveledMeters = (finalX - initialX) * 0.0254;

            double initialHeightMeters = ShootingMath.calculateExactExitHeightMeters(controls.ballExitAngleRad);
            double finalHeightMeters = ShootingMath.shooterSystemParams.ballRadiusMeters + (controls.heightToShootBallInches * 0.0254);
            changeInYMeters = finalHeightMeters - initialHeightMeters;

            double targetVelMetersPerSec = calculateTargetExitVelocity(totalDistanceTraveledMeters, changeInYMeters, controls.ballExitAngleRad);

            // vel target = vel actual * getEfficiency(vel target)
            // vel actual = vel target / getEfficiency(vel target)
            double targetExitVelTicksPerSec = ShootingMath.exitMpsToMotorTicksPerSec(targetVelMetersPerSec, 1);
            double efficiencyCoefficient = controls.efficiencyCoefficient;
            if (efficiencyCoefficient < 0)
                //y=-0.002x+0.72
                efficiencyCoefficient = -0.002 * Math.toDegrees(controls.ballExitAngleRad) + 0.72;
            double actualVelTicksPerSec = targetExitVelTicksPerSec / efficiencyCoefficient;

            robot.shooter.setShooterVelocityPID(actualVelTicksPerSec, shooterVelTicksPerSec);

            telemetry.addData("a exit pos X inches", exitPosition.x);
            telemetry.addData("b distance meters", totalDistanceTraveledMeters);
            telemetry.addData("c change in Y from ball exit position (meters)", changeInYMeters);
            telemetry.addData("d ball exit angle degrees", Math.toDegrees(controls.ballExitAngleRad));
            telemetry.addData("e efficiency coefficient", efficiencyCoefficient);
            telemetry.addData("f target exit velocity meters per sec", targetVelMetersPerSec);
            telemetry.addData("g target exit velocity ticks per sec", targetExitVelTicksPerSec);
            telemetry.addData("h actual shooter velocity ticks per sec", actualVelTicksPerSec);
            telemetry.addData("i current shooter vel ticks per sec", shooterVelTicksPerSec);
            telemetry.update();
        }
    }

    private double calculateTargetExitVelocity(double distanceMeters, double changeInYMeters, double exitAngleRad) {
        double cosTheta = Math.cos(exitAngleRad);
        double tanTheta = Math.tan(exitAngleRad);
        double denominator = 2 * cosTheta * cosTheta * (distanceMeters * tanTheta - changeInYMeters);
        if (denominator <= 0)
            return -1;
        double numerator = experiment.gravityAcceleration * distanceMeters * distanceMeters;
        return Math.sqrt(numerator / denominator);
    }
}
