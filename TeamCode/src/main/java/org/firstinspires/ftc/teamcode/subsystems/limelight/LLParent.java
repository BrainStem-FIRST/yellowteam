package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;

public abstract class LLParent {
    protected final Limelight3A limelight;
    protected final BrainSTEMRobot robot;

    public LLParent(BrainSTEMRobot robot, Limelight3A limelight) {
        this.robot = robot;
        this.limelight = limelight;
    }

    public abstract void update();
    public abstract void updateTelemetry(Telemetry telemetry);
}
