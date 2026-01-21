package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;


import java.util.ArrayList;


@Config
public class BrainSTEMRobot {
    public static double width = 13, length = 16; // inches
    public static boolean enablePinpoint = true, enableSubsystems = true;
    public static boolean enableTurret = true, enableShooter = true, enableCollection = true, enableLimelight = true, enablePark = true, enableLED = true;

    public Turret turret;
    public Shooter shooter;
    public Collection collection;
    public Parking parking;
    public MecanumDrive drive;
    public Limelight limelight;
    public LED led;
    public static Alliance alliance;
    private final ArrayList<Component> subsystems;
    private final Telemetry telemetry;
    public GamepadTracker g1;

    public BrainSTEMRobot(Alliance alliance, Telemetry telemetry, HardwareMap hardwareMap, Pose2d initialPose){
        this.telemetry = telemetry;
        BrainSTEMRobot.alliance = alliance;
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
        if(enablePinpoint)
            drive.updatePoseEstimate();
        turret.updateLookAheadTime(useTurretLookAhead);
        shooter.updateProperties();


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
        Pose2d exitPose = new Pose2d(exitPosition, turret.currentAbsoluteAngleRad);

        fieldOverlay.setStroke("red");
        Drawing.drawRobot(fieldOverlay, robotPose);
        fieldOverlay.setStroke("green");
        Drawing.drawRobotSimple(fieldOverlay, turretPose, 5);
        fieldOverlay.setStroke("purple");
        Drawing.drawRobotSimple(fieldOverlay, exitPose, 3);
        fieldOverlay.setStroke("blue");
        Drawing.drawRobotSimple(fieldOverlay, new Pose2d(shooter.prevExitPos, 0), 3);

        limelight.addLimelightInfo(fieldOverlay);

        // draw where turret is pointed
        fieldOverlay.setAlpha(1);
        double dist = Math.hypot(exitPosition.x - turret.targetPose.position.x, exitPosition.y - turret.targetPose.position.y);

        fieldOverlay.setStroke("purple");
        fieldOverlay.strokeLine(
                exitPosition.x,
                exitPosition.y,
                exitPosition.x + dist * Math.cos(turret.currentAbsoluteAngleRad),
                exitPosition.y + dist * Math.sin(turret.currentAbsoluteAngleRad)
        );
        fieldOverlay.setStroke("black");
        fieldOverlay.strokeLine(
                exitPosition.x,
                exitPosition.y,
                exitPosition.x + dist * Math.cos(turret.absoluteTargetAngleRad),
                exitPosition.y + dist * Math.sin(turret.absoluteTargetAngleRad)
        );
        double speedMag = Math.hypot(ShootingMath.relativeBallExitVelocityMps.x, ShootingMath.relativeBallExitVelocityMps.y);
        fieldOverlay.setStroke("red");
        fieldOverlay.strokeLine(
                exitPosition.x,
                exitPosition.y,
                exitPosition.x + 1.5 * speedMag * Math.cos(turret.absoluteTargetAngleRad),
                exitPosition.y + 1.5 * speedMag * Math.sin(turret.absoluteTargetAngleRad)
        );
    }
}
