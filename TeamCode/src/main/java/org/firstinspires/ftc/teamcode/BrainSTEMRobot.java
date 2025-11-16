package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.LED;
import org.firstinspires.ftc.teamcode.subsystems.Parking;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;


import java.util.ArrayList;


@Config
public class BrainSTEMRobot implements Component {

    public static boolean enableTurret = true, enableShooter = true, enableCollection = true, enableVision = true, enablePark = true, enableLED = true;

    public Turret turret;
    public Shooter shooter;
    public Collection collection;
    public Parking parking;
    public MecanumDrive drive;
    public Vision vision;
    public LED led;
    private final ArrayList<Component> subsystems;

    public BrainSTEMRobot(Telemetry telemetry, HardwareMap hardwareMap, Pose2d initialPose){
        subsystems = new ArrayList<>();

        drive = new MecanumDrive(hardwareMap, initialPose);
        vision = new Vision(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry, this);
        shooter = new Shooter(hardwareMap, telemetry, drive, turret);
        collection = new Collection(hardwareMap, telemetry);
        parking = new Parking(hardwareMap, telemetry, drive);
        led = new LED(hardwareMap, telemetry, shooter, turret, parking, collection);

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

    @Override
    public void reset() {
        for (Component c : subsystems) {
            c.reset();
        }
    }

    @Override
    public void update(){
        drive.updatePoseEstimate();

        for (Component c : subsystems) {
            c.update();
        }

//        drawRobot(this);
//        telemetry.update();
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
