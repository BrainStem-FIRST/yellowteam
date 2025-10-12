package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TurretTrackingCommand extends CommandBase {
    BrainSTEMRobot robot;
    Telemetry telemetry;
    Pose2d pose;

    public TurretTrackingCommand(BrainSTEMRobot robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = telemetry;
        this.pose = pose;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        robot.turret.turretState = Turret.TurretState.TRACKING;
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
