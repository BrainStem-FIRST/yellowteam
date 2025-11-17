package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


import java.util.ArrayList;


@Config
public class BrainSTEMRobot {

    public static boolean enableTurret = true, enableShooter = true, enableCollection = true, enableVision = true, enablePark = true, enableLED = true;

    public Turret turret;
    public Shooter shooter;
    public Collection collection;
    public Parking parking;
    public MecanumDrive drive;
    public Vision vision;
    public LED led;
    public final Alliance alliance;
    private final ArrayList<Component> subsystems;

    public BrainSTEMRobot(Alliance alliance, Telemetry telemetry, HardwareMap hardwareMap, Pose2d initialPose){
        this.alliance = alliance;
        subsystems = new ArrayList<>();

        drive = new MecanumDrive(hardwareMap, initialPose);
        vision = new Vision(hardwareMap, telemetry, this);
        turret = new Turret(hardwareMap, telemetry, this);
        shooter = new Shooter(hardwareMap, telemetry, this);
        collection = new Collection(hardwareMap, telemetry, this);
        parking = new Parking(hardwareMap, telemetry, this);
        led = new LED(hardwareMap, telemetry, this);

        if (enableTurret)
            subsystems.add(turret);
        if (enableShooter)
            subsystems.add(shooter);
        if (enableCollection)
            subsystems.add(collection);
        if (enablePark)
            subsystems.add(parking);
        if (enableVision)
            subsystems.add(vision);
        if (enableLED)
            subsystems.add(led);
    }
    public void reset() {
        for (Component c : subsystems) {
            c.reset();
        }
    }
    public void update(){
        drive.updatePoseEstimate();

        for (Component c : subsystems) {
            c.update();
        }

//        drawRobot(this);
    }

    private void drawRobot(BrainSTEMRobot robot) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
