package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Component {
    protected final HardwareMap hardwareMap;
    protected final Telemetry telemetry;
    protected final BrainSTEMRobot robot;

    public Component(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    public abstract void printInfo();

    public abstract void reset();

    public abstract void update();
}
