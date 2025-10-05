package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TurretTrackingCommand extends CommandBase {
    BrainSTEMRobot robot;
    Turret turret;
    Telemetry telemetry;
    Pose2d pose;


    public TurretTrackingCommand(BrainSTEMRobot robot, Telemetry telemetry, Turret turret, Pose2d pose){
        this.robot = robot;
        this.telemetry = telemetry;
        this.turret = turret;
        this.pose = pose;
    }

    @Override
    public void initialize() {
        turret.setTurretTracking();
    }


    public void execute(){
        telemetry.addData("turret encoder", turret.getTurretEncoder());
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
