package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;

public class EngageClutchCommand extends CommandBase {
    BrainSTEMRobot robot;
    Telemetry telemetry;

    public EngageClutchCommand(BrainSTEMRobot robot, Telemetry telemetry) {
        this.robot = robot;
    }

    @Override
    public void execute() {
        robot.collection.clutchState = Collection.ClutchState.ENGAGED;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
