package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoPositions;

public class BlueFarAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(Alliance.BLUE, telemetry, hardwareMap, AutoPositions.blueFarStart);
        AutoPositions autoPositions = new AutoPositions(robot.drive);
        waitForStart();
        while(opModeIsActive()) {

        }
    }
}
