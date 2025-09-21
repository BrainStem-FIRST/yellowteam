package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


import java.util.ArrayList;


public class BrainSTEMRobot implements Component {

    // Initialization
    private LinearOpMode opMode;
    private Telemetry telemetry;
    private HardwareMap map;


    // components here
    public Turret turret;
    public MecanumDrive drive;


    //List of components to be initialized
    private ArrayList<Component> subsystems;


    public BrainSTEMRobot(Telemetry telemetry, HardwareMap hardwareMap, Pose2d initialPose){
        this.telemetry = telemetry;
        this.map = hardwareMap;

        // init components
        turret = new Turret(map, telemetry);
        drive = new MecanumDrive(map, initialPose);

        // add components to list here
        subsystems.add(turret);
    }

    public void getTelemetry(){

    }

    @Override
    public void reset() {

    }

    @Override
    public void update(){
        drive.updatePoseEstimate();
        drawRobot(this);
        telemetry.update();
    }

    private void drawRobot(BrainSTEMRobot robot) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public String test() {

        return null;
    }
}
