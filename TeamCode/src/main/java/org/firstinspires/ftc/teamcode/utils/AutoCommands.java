package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.commands.TurretTrackingCommand;

public class AutoCommands {
    BrainSTEMRobot robot;
    Telemetry telemetry;

    public AutoCommands(BrainSTEMRobot robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public class EnableTurretTracking implements Action {
        private boolean initialized = false;
        private TurretTrackingCommand turretTrackingCommand;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                turretTrackingCommand = new TurretTrackingCommand(robot, telemetry);
                turretTrackingCommand.execute();
                initialized = true;
            }
            robot.update();
            return !turretTrackingCommand.isFinished();
        }
    }

    public Action enableTurretTracking(){
        return new EnableTurretTracking();
    }


}