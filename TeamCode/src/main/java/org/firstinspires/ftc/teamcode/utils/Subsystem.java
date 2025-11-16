package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.Component;

public abstract class Subsystem implements Component {
    protected final HardwareMap hardwareMap;
    protected final Telemetry telemetry;
    protected final BrainSTEMRobot robot;

    public Subsystem(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.robot = robot;
    }

    public abstract void printInfo();
}
