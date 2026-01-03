package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryHelper;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;


import java.util.ArrayList;


@Config
public class BrainSTEMRobot {
    // TODO: actually find these values
    public static double width = 10, length = 18; // inches
    public static boolean enablePinpoint = true, enableSubsystems = true;
    public static boolean enableTurret = true, enableShooter = true, enableCollection = true, enableLimelight = true, enablePark = true, enableLED = true;

    public Turret turret;
    public Shooter shooter;
    public Collection collection;
    public Parking parking;
    public MecanumDrive drive;
    public Limelight limelight;
    public LED led;
    public final Alliance alliance;
    private final ArrayList<Component> subsystems;
    private final Telemetry telemetry;
    public GamepadTracker g1;

    public BrainSTEMRobot(Alliance alliance, Telemetry telemetry, HardwareMap hardwareMap, Pose2d initialPose){
        this.telemetry = telemetry;
        this.alliance = alliance;
        subsystems = new ArrayList<>();

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
        if (enableLED)
            subsystems.add(led);
    }
    public void setG1(GamepadTracker g1) {
        this.g1 = g1;
    }
    public void update(boolean useTurretLookAhead) {
        turret.updateLookAheadTime(useTurretLookAhead);

        if(enablePinpoint)
            drive.updatePoseEstimate();

        Pose2d pose = drive.localizer.getPose();

        telemetry.addData("pose", MathUtils.format3(pose.position.x) + ", " + MathUtils.format3(pose.position.y) + " | " + MathUtils.format3(pose.heading.toDouble()));
        if(enableSubsystems)
            for (Component c : subsystems)
                c.update();
    }

    public void addRobotInfo(Canvas fieldOverlay) {
        // draw robot, turret, exit position, and limelight pose
        Pose2d robotPose = drive.pinpoint().getPose();
        int turretEncoder = turret.getTurretEncoder();
        Pose2d turretPose = Turret.getTurretPose(robotPose, turretEncoder);
        Vector2d exitPosition = ShootingMath.calculateExitPositionInches(robotPose, turretEncoder, shooter.getBallExitAngleRad());
        Pose2d exitPose = new Pose2d(exitPosition, turret.currentAngleRad);


        TelemetryHelper.colors[0] = "red";
        TelemetryHelper.colors[1] = "green";
        TelemetryHelper.colors[2] = "purple";
        TelemetryHelper.radii[0] = 10;
        TelemetryHelper.radii[1] = 6;
        TelemetryHelper.radii[2] = 3;
        TelemetryHelper.numPosesToShow = 3;
        TelemetryHelper.addRobotPoseToCanvas(fieldOverlay, robotPose, turretPose, exitPose);

        limelight.addLimelightInfo(fieldOverlay);

        // draw where turret is pointed
        fieldOverlay.setAlpha(1);
        double dist = Math.hypot(exitPosition.x - turret.targetPose.position.x, exitPosition.y - turret.targetPose.position.y);

        fieldOverlay.setStroke("purple");
        fieldOverlay.strokeLine(
                exitPosition.x,
                exitPosition.y,
                exitPosition.x + dist * Math.cos(turret.currentAngleRad),
                exitPosition.y + dist * Math.sin(turret.currentAngleRad)
        );
        fieldOverlay.setStroke("black");
        fieldOverlay.strokeLine(
                exitPosition.x,
                exitPosition.y,
                exitPosition.x + dist * Math.cos(turret.targetAngleRad),
                exitPosition.y + dist * Math.sin(turret.targetAngleRad)
        );
    }
}
