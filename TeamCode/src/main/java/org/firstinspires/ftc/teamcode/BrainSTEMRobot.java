package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


import java.util.ArrayList;


public class BrainSTEMRobot implements Component {

    private LinearOpMode opMode;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public Turret turret;
    public Shooter shooter;
    public MecanumDrive drive;

    private ArrayList<Component> subsystems;

    public BrainSTEMRobot(Telemetry telemetry, HardwareMap hardwareMap, Pose2d initialPose){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        subsystems = new ArrayList<>();

        turret = new Turret(hardwareMap, telemetry);
        shooter = new Shooter(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, initialPose);

        subsystems.add(turret);
        subsystems.add(shooter);
    }

    public void getTelemetry(){

    }

    @Override
    public void reset() {
        for (Component c : subsystems) {
            c.reset();
        }
    }

    @Override
    public void update(){
        for (Component c : subsystems) {
            c.update();
        }

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
