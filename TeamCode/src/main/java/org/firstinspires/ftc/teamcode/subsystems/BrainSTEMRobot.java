package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.Vec;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;


import java.util.ArrayList;
import java.util.List;


@Config
public class BrainSTEMRobot {
    public static boolean enablePinpoint = true, enableSubsystems = true;
    public static boolean enableTurret = true, enableShooter = true, enableCollection = true, enableLimelight = true, enablePark = true, enableLED = true;

    public Turret turret;
    public Shooter shooter;
    public Collection collection;
    public Parking parking;
    public MecanumDrive drive;
//    public Vision vision;
    public Limelight limelight;
    public LED led;
    public final Alliance alliance;
    private final ArrayList<Component> subsystems;
    private final List<LynxModule> allHubs;
    private Telemetry telemetry;
    public GamepadTracker g1;

    public BrainSTEMRobot(Alliance alliance, Telemetry telemetry, HardwareMap hardwareMap, Pose2d initialPose){
        this.telemetry = telemetry;
        this.alliance = alliance;
        subsystems = new ArrayList<>();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        drive = new MecanumDrive(hardwareMap, initialPose);
        limelight = new Limelight(hardwareMap, telemetry, this);
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
        if (enableLimelight)
            subsystems.add(limelight);
//        subsystems.add(vision);
        if (enableLED)
            subsystems.add(led);
    }
    public void setG1(GamepadTracker g1) {
        this.g1 = g1;
    }
    public void reset() {
        for (Component c : subsystems) {
            c.reset();
        }
    }
    public void update(boolean useTurretLookAhead) {
        for(LynxModule hub : allHubs)
            hub.clearBulkCache();


        turret.updateLookAheadTime(useTurretLookAhead);

        if(enablePinpoint)
            drive.updatePoseEstimate();

        Pose2d pose = drive.localizer.getPose();

        telemetry.addData("pose", MathUtils.format3(pose.position.x) + ", " + MathUtils.format3(pose.position.y) + " | " + MathUtils.format3(pose.heading.toDouble()));
        if(enableSubsystems)
            for (Component c : subsystems)
                c.update();
    }

    private void drawRobot(BrainSTEMRobot robot) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
