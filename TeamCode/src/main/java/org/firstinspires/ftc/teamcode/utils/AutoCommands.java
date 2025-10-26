package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
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

    // OTHER
    public Action setBlueAlliance() {
        return packet -> {
            robot.turret.isRedAlliance = false;
            return false;
        };
    }

    public Action setRedAlliance() {
        return packet -> {
            robot.turret.isRedAlliance = true;
            return false;
        };
    }


    // TURRET
    public Action enableTurretTracking() {
        return packet -> {
            robot.turret.turretState = Turret.TurretState.TRACKING;
            return false;
        };
    }

    // SHOOTER
    public Action spinUpShooter() {
        return packet -> {
            robot.shooter.shooterState = Shooter.ShooterState.UPDATE;
            return robot.shooter.shooterMotorHigh.getVelocity() < 1375;
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