package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShootingSystem.DistState.FAR;
import static org.firstinspires.ftc.teamcode.subsystems.ShootingSystem.DistState.NEAR;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;


@Config
public class BrainSTEMRobot {
    public static double width = 13, length = 16; // inches
    public static boolean enablePinpoint = true;
    public static boolean enableTurret = true, enableShooter = true, enableCollection = true, enableLimelight = true, enableLED = true;
    public int lookAheadAvgNum = 5;
    public double rawLookAheadTime = 0.25; // time to look ahead for pose prediction

    public final ShootingSystem shootingSystem;
    public final Collection collection;
    public final Parking parking;
    public final MecanumDrive drive;
    public final Limelight limelight;
    public final LED led;
    public static Alliance alliance;
    private final Telemetry telemetry;
    public GamepadTracker g1;
    private ElapsedTime timer;
    private double dt;
    private double currentLookAhead;
    private final double[] prevLookAheads;
    private boolean shootingWhileMoving;

    public BrainSTEMRobot(Alliance alliance, Telemetry telemetry, HardwareMap hardwareMap, Pose2d initialPose){
        this.telemetry = telemetry;
        BrainSTEMRobot.alliance = alliance;

        prevLookAheads = new double[lookAheadAvgNum];
        for(int i = 0; i < lookAheadAvgNum; i++)
            prevLookAheads[i] = 0;

        drive = new MecanumDrive(hardwareMap, initialPose);
        limelight = new Limelight(hardwareMap, telemetry, this);
        shootingSystem = new ShootingSystem(hardwareMap, telemetry, initialPose);
        collection = new Collection(hardwareMap, telemetry);
        parking = new Parking(hardwareMap, telemetry);
        led = new LED(hardwareMap, telemetry);

        timer = new ElapsedTime();
        timer.reset();
    }
    public void setG1(GamepadTracker g1) {
        this.g1 = g1;
    }
    public void update(boolean useLookAhead) {
        dt = timer.seconds();
        timer.reset();

        if(enablePinpoint) {
            drive.updatePoseEstimate();
        }
        Pose2d robotPose = drive.localizer.getPose();
        updateLookAheadTime(useLookAhead);
        Pose2d futureRobotPose = drive.pinpoint().getNextPoseSimple(currentLookAhead);

        if(enableCollection) {
            collection.updateProperties();
            boolean shotPossible = false; // TODO: figure this boolean out
            collection.updateState(shotPossible);
        }
        shootingWhileMoving = (ShootingSystem.testingParams.enableShootingWhileMovingFar && shootingSystem.getDistState() == FAR || ShootingSystem.testingParams.enableShootingWhileMovingNear && shootingSystem.getDistState() == NEAR) && collection.getClutchState() == Collection.ClutchState.ENGAGED && shootingSystem.getTurret().inRange();
        shootingSystem.updateProperties(dt, robotPose, futureRobotPose, drive.pinpoint().getMostRecentVelocity(), shootingWhileMoving);
        shootingSystem.updateState(dt, enableShooter, enableTurret);

        if(enableLimelight) {
            limelight.update();
            if(LimelightLocalization.localizationType == LimelightLocalization.LocalizationType.CONTINUOUS && limelight.localization.robotPose != null)
                drive.pinpoint().setPose(limelight.localization.robotPose);
        }
        if(enableLED) {
            boolean updatingPose = limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE;
            boolean confirmingPose = false; // TODO: figure this boolean out too
            led.update(updatingPose, confirmingPose, shootingSystem.getShooter().inTolerance(shootingSystem.getTargetShooterSpeedTps()), shootingSystem.getTurret().inRange(), collection.getClutchState() == Collection.ClutchState.ENGAGED, collection.getCollectionState() == Collection.CollectionState.INTAKE, collection.intakeHas3Balls());
        }
    }

    private void updateLookAheadTime(boolean useLookAhead) {
        double rawLookAhead = useLookAhead ? rawLookAheadTime : 0;
        for(int i = prevLookAheads.length-1; i > 0; i--)
            prevLookAheads[i] = prevLookAheads[i-1];
        prevLookAheads[0] = rawLookAhead;
        currentLookAhead = getAvgLookAheads();
    }
    private double getAvgLookAheads() {
        double sum = 0;
        for (double prevLookAhead : prevLookAheads) sum += prevLookAhead;
        return sum / prevLookAheads.length;
    }

    public void addRobotInfo(Canvas fieldOverlay) {
        // draw robot, turret, exit position, and limelight pose
        Pose2d robotPose = drive.pinpoint().getPose();

        fieldOverlay.setStroke("red");
        Drawing.drawRobot(fieldOverlay, robotPose);
        fieldOverlay.setStroke("green");
        Drawing.drawRobotSimple(fieldOverlay, shootingSystem.getAbsoluteTurretPose(), 5);
        fieldOverlay.setStroke("purple");
        Drawing.drawRobotSimple(fieldOverlay, new Pose2d(shootingSystem.getBallExitPos(), 0), 3);

        limelight.addLimelightInfo(fieldOverlay);

        // draw where turret is pointed
        fieldOverlay.setAlpha(1);
        double dist = Math.hypot(shootingSystem.getBallExitPos().x - shootingSystem.get2dGoalPos().x, shootingSystem.getBallExitPos().y - shootingSystem.get2dGoalPos().y);

        fieldOverlay.setStroke("purple");
        double curAbsAng = shootingSystem.getTurret().getAbsAngleRad(robotPose.heading.toDouble());
        fieldOverlay.strokeLine(
                shootingSystem.getBallExitPos().x,
                shootingSystem.getBallExitPos().y,
                shootingSystem.getBallExitPos().x + dist * Math.cos(curAbsAng),
                shootingSystem.getBallExitPos().y + dist * Math.sin(curAbsAng)
        );
        fieldOverlay.setStroke("black");
        fieldOverlay.strokeLine(
                shootingSystem.getBallExitPos().x,
                shootingSystem.getBallExitPos().y,
                shootingSystem.getBallExitPos().x + dist * Math.cos(shootingSystem.getTurretAbsoluteTargetAngle()),
                shootingSystem.getBallExitPos().y + dist * Math.sin(shootingSystem.getTurretAbsoluteTargetAngle())
        );
    }
    public double getDt() {
        return dt;
    }
    public boolean isShootingWhileMoving() {
        return shootingWhileMoving;
    }
}
