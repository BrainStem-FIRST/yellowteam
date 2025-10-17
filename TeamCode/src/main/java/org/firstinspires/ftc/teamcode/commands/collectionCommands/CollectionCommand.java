package org.firstinspires.ftc.teamcode.commands.collectionCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Collection;

public class CollectionCommand extends CommandBase {
    BrainSTEMRobot robot;
    Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    public CollectionCommand(BrainSTEMRobot robot, Telemetry telemetry) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        timer.reset();
        robot.collection.startIntake();
    }

    @Override
    public void execute() {
        robot.collection.updateIntakeSequence(timer.seconds());
    }

    @Override
    public boolean isFinished() {
        return robot.collection.collectionState == Collection.CollectionState.OFF;
    }

    @Override
    public void end(boolean interrupted) {
        robot.collection.stopIntake(); //remove if interruption is ok
    }
}
