package org.firstinspires.ftc.teamcode.commandGroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.commands.CollectionCommand;
import org.firstinspires.ftc.teamcode.commands.EngageClutchCommand;

public class FullCollectionSequence extends SequentialCommandGroup {
    public FullCollectionSequence(BrainSTEMRobot robot, Telemetry telemetry) {
        super(
                new CollectionCommand(robot, telemetry),
                new WaitCommand(250),
                new EngageClutchCommand(robot, telemetry)
        );
    }
}