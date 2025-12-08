package org.firstinspires.ftc.teamcode.utils.autoHelpers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.opmode.teleop.BrainSTEMTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;

public class AutoCommands {
    BrainSTEMRobot robot;
    Telemetry telemetry;

    public AutoCommands(BrainSTEMRobot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public Action waitTillDoneShooting(double maxTimeBetweenShots, double intakeCurrentValidationTime) {
        return new Action() {
            private final ElapsedTime timer = new ElapsedTime();
            private final ElapsedTime timeSinceLastVelDrop = new ElapsedTime();
            private int oldBallsShot;
            private boolean first = true;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    first = false;
                    timer.reset();
                    timeSinceLastVelDrop.reset();
                    oldBallsShot = robot.shooter.getBallsShot();
                }
                if (robot.collection.collectorMotor.getPower() != Collection.COLLECTOR_PARAMS.INTAKE_SPEED) {
                    timer.reset();
                    timeSinceLastVelDrop.reset();
                }

                if (robot.collection.collectorMotor.getCurrent(CurrentUnit.MILLIAMPS) > Collection.COLLECTOR_PARAMS.hasBallCurrentThreshold)
                    timer.reset();

                if (oldBallsShot != robot.shooter.getBallsShot())
                    timeSinceLastVelDrop.reset();

                oldBallsShot = robot.shooter.getBallsShot();
                telemetry.addData("time since last vel drop", timeSinceLastVelDrop.seconds());


                return timeSinceLastVelDrop.seconds() < maxTimeBetweenShots && timer.seconds() < intakeCurrentValidationTime && robot.shooter.getBallsShot() < 3;
            }
        };
    }
    // CONSTANT UPDATES
    public Action updateRobot = packet -> {
        robot.update(true);
        return true;
    };

    public Action savePoseContinuously = packet -> {
        Pose2d cur = robot.drive.localizer.getPose();
        PoseStorage.autoX = cur.position.x;
        PoseStorage.autoY = cur.position.y;
        PoseStorage.autoHeading = cur.heading.toDouble();
        return true;
    };

    // TURRET
    public Action enableTurretTracking() {
        return packet -> {
            robot.turret.turretState = Turret.TurretState.TRACKING;
            return false;
        };
    }

    public Action turretCenter() {
        return packet -> {
            robot.turret.turretState = Turret.TurretState.CENTER;
            return false;
        };
    }

    // SHOOTER
    public Action speedUpShooter() {
        return packet -> {
            robot.shooter.shooterState = Shooter.ShooterState.UPDATE;
            return Math.abs(robot.shooter.getAvgMotorVelocity() - robot.shooter.shooterPID.getTarget()) >= BrainSTEMTeleOp.firstShootTolerance;
        };
    }

    public Action maxShooterSpeed() {
        return packet -> {
            robot.shooter.setShooterPower(1);
            return false;
        };
    }

    public Action stopShooter() {
        return packet -> {
            robot.shooter.shooterState = Shooter.ShooterState.OFF;
            return false;
        };
    }

    // COLLECTIONS
    public Action engageClutchAndIntake() {
        return new ParallelAction(
                engageClutch(),
                runIntake()
        );
    }
    public Action engageClutch() {
        return packet -> {
            robot.collection.clutchState = Collection.ClutchState.ENGAGED;
            robot.collection.extakeAfterClutchEngage = false;
            robot.collection.clutchStateTimer.reset();
            robot.collection.clutch_timer.reset();
            return false;
        };
    }

    public Action disengageClutch() {
        return packet -> {
            robot.collection.clutchState = Collection.ClutchState.UNENGAGED;
            robot.collection.clutchStateTimer.reset();
            return false;
        };
    }

    public Action flickerUp() {
        return packet -> {
            robot.collection.flickerState = Collection.FlickerState.FULL_UP_DOWN;
            robot.collection.collectionState = Collection.CollectionState.OFF;
            return false;
        };
    }
    public Action flickerHalfUp() {
        return packet -> {
            robot.collection.flickerState = Collection.FlickerState.HALF_UP_DOWN;
            return false;
        };
    }

    public Action flickerDown() {
        return packet -> {
            robot.collection.flickerState = Collection.FlickerState.DOWN;
            return false;
        };
    }


    public Action runIntake() {
        return packet -> {
            robot.collection.collectionState = Collection.CollectionState.INTAKE;
            return false;
        };
    }
    public Action intakeSlow() {
        return telemetryPacket -> {
            robot.collection.collectionState = Collection.CollectionState.INTAKE_SLOW;
            return false;
        };
    }

    public Action reverseIntake() {
        return packet -> {
            robot.collection.collectionState = Collection.CollectionState.OUTTAKE;
            return false;
        };
    }

    public Action stopIntake() {
        return packet -> {
            robot.collection.collectionState = Collection.CollectionState.OFF;
            return false;
        };
    }
}