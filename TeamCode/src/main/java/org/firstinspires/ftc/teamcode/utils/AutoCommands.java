package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class AutoCommands {
    BrainSTEMRobot robot;
    Telemetry telemetry;

    public AutoCommands(BrainSTEMRobot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    // CONSTANT UPDATES
    public Action updateRobot = packet -> {
        robot.update();
        return true;
    };

    public Action savePoseContinuously = packet -> {
        PoseStorage.currentPose = robot.drive.localizer.getPose();
        return true;
    };

    public Action saveTurretContinuously = packet -> {
        PoseStorage.currentTurretEncoder = robot.turret.getTurretEncoder();
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
    public Action spinUpShooter(boolean isClose) {
        return packet -> {
            robot.shooter.shooterState = Shooter.ShooterState.UPDATE;
            return -robot.shooter.shooterMotorHigh.getVelocity() < 1050;
        };
    }

    public Action setShooterVelocity() {
        return packet -> {
            robot.shooter.shooterState = Shooter.ShooterState.AUTO_VELOCITY;
            return -robot.shooter.shooterMotorHigh.getVelocity() < 1050;
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
    public Action engageClutch() {
        return packet -> {
            robot.collection.clutchState = Collection.ClutchState.ENGAGED;
            return false;
        };
    }

    public Action disengageClutch() {
        return packet -> {
            robot.collection.clutchState = Collection.ClutchState.UNENGAGED;
            return false;
        };
    }

    public Action flickerUp() {
        return packet -> {
            robot.collection.flickerState = Collection.FlickerState.UP_DOWN;
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

    public Action reverseIntake() {
        return packet -> {
            robot.collection.collectionState = Collection.CollectionState.EXTAKE;
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