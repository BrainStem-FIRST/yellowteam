package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.utils.math.MathUtils.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.Vec;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;

import java.util.List;

@Config
public class BrainSTEMTeleOp extends LinearOpMode {
    public static boolean printCollector = false, printShooter = true, printTurret = false, printShootingSystem = true, printLimelight = false;
    public static double[] blueCornerResetPose = { 64.25, 62.75, -90 };
    public static double[] redCornerResetPose = { 64.25, -62.75, 90 };
    public static double firstShootTolerance = 40;

    public enum PosePredictType {
        SIMPLE,
        ADVANCED,
        CONTROL
    }
    public static double noMoveJoystickThreshold = 0.1;

    BrainSTEMRobot robot;

    GamepadTracker gp1;
    GamepadTracker gp2;

    private Pose2d lastFrameSimplePrediction, lastFrameAdvancedPrediction;
    private final Alliance alliance;
    private boolean currentlyMoving;
    private List<LynxModule> allHubs;

    public BrainSTEMTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() {

//        allHubs = hardwareMap.getAll(LynxModule.class);
//        for(LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        Pose2d startPose = new Pose2d(PoseStorage.autoX, PoseStorage.autoY, PoseStorage.autoHeading);
        lastFrameSimplePrediction = startPose;
        lastFrameAdvancedPrediction = startPose;
        currentlyMoving = false;

        telemetry.setMsTransmissionInterval(20); // faster telemetry speed
        CommandScheduler.getInstance().reset();

        robot = new BrainSTEMRobot(alliance, telemetry, hardwareMap, startPose); //take pose from auto
        gp1 = new GamepadTracker(gamepad1);
        gp2 = new GamepadTracker(gamepad2);
        robot.setG1(gp1);
        telemetry.addData("starting pose", MathUtils.formatPose3(startPose));
        if (!robot.limelight.limelight.isConnected())
            telemetry.addLine("WARNING - LIMELIGHT IS NOT CONNECTED");
        if (!robot.limelight.limelight.isRunning())
            telemetry.addLine("WARNING - LIMELIGHT IS NOT RUNNING");
        telemetry.update();

        waitForStart();
        int framesRunning = 0;
        long startTimeNano = System.nanoTime();

        robot.turret.update();
        while (opModeIsActive()) {
            gp1.update();
            gp2.update();

            //telemetry.addData("TRACKING SHOOTER DATA", robot.shooter.isTrackingData());
            //telemetry.addLine();
            //telemetry.addData("SHOOTER ADJUSTMENT", robot.shooter.adjustment);
            //telemetry.addData("TURRET ADJUSTMENT", robot.turret.adjustment);
            //telemetry.addLine();

            updateDrive();
            updateDriver2();
            updateDriver1();
            CommandScheduler.getInstance().run();

            robot.update(currentlyMoving);

            telemetry.addData("Alliance", BrainSTEMRobot.alliance);

            if (printCollector)
                robot.collection.printInfo();
            if (printLimelight)
                robot.limelight.printInfo();
            if (printTurret)
                robot.turret.printInfo();
            if (printShooter)
                robot.shooter.printInfo();
            if(printShootingSystem)
                robot.shootingSystem.printInfo(telemetry);

            updateDashboardField();

            // print delta time
            framesRunning++;
//            double timeRunning = (System.nanoTime() - startTimeNano) * 1.0 * 1e-9;
            if(gp1.isFirstStart()) {
                framesRunning = 0;
                startTimeNano = System.nanoTime();
            }
//            telemetry.addData("FPS", MathUtils.format2(framesRunning / timeRunning));
            telemetry.update();

            Pose2d p = robot.drive.localizer.getPose();
            PoseStorage.autoX = p.position.x;
            PoseStorage.autoY = p.position.y;
            PoseStorage.autoHeading = p.heading.toDouble();
        }
    }

    private void updateDrive() {
        if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE && robot.limelight.localization.manualPoseUpdate) {
            stop();
            return;
        }
        currentlyMoving = Math.abs(gamepad1.left_stick_x) > noMoveJoystickThreshold || Math.abs(gamepad1.left_stick_y) > noMoveJoystickThreshold || Math.abs(gamepad1.right_stick_x) > noMoveJoystickThreshold;
        robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));
    }

    private void updateDriver1() {
        if(robot.collection.getClutchState() == Collection.ClutchState.UNENGAGED) {
            if (gp1.gamepad.right_trigger > 0.2)
                robot.collection.setCollectionState(Collection.CollectionState.INTAKE);
            else if (gp1.gamepad.left_trigger > 0.2)
                robot.collection.setCollectionState(Collection.CollectionState.OUTTAKE);
            else
                robot.collection.setCollectionState(Collection.CollectionState.OFF);
        }

        if (gp1.isFirstB()) {
            if (robot.collection.getClutchState() == Collection.ClutchState.ENGAGED)
                robot.collection.setClutchState(Collection.ClutchState.UNENGAGED);
            else
                robot.collection.setClutchState(Collection.ClutchState.ENGAGED);
        }

        if (gp1.isFirstRightBumper())
            if (robot.shooter.shooterState == Shooter.ShooterState.UPDATE)
                robot.shooter.shooterState = Shooter.ShooterState.OFF;
            else
                robot.shooter.shooterState = Shooter.ShooterState.UPDATE;

        if (gp1.isFirstLeftBumper()) {
            if (robot.turret.turretState == Turret.TurretState.CENTER)
                robot.turret.turretState = Turret.TurretState.TRACKING;
            else
                robot.turret.turretState = Turret.TurretState.CENTER;
        }

        if (gp1.isFirstBack()) {
            robot.limelight.localization.maxTranslationalVariance = 0;
            robot.limelight.localization.maxHeadingVarianceDeg = 0;
            robot.limelight.localization.maxTranslationalError = 0;
            robot.limelight.localization.maxHeadingErrorDeg = 0;
        }
    }

    private void updateDriver2() {
        if(robot.collection.getClutchState() == Collection.ClutchState.ENGAGED) {
            if (gp2.isFirstA())
                if (robot.collection.getCollectionState() == Collection.CollectionState.INTAKE)
                    robot.collection.setCollectionState(Collection.CollectionState.OFF);
                else if (Math.abs(robot.shootingSystem.filteredShooterSpeedTps - robot.shooter.shooterPID.getTarget()) <= firstShootTolerance && robot.turret.inRange())
                    robot.collection.setCollectionState(Collection.CollectionState.INTAKE);
        }
        if (gp2.isFirstB())
            if (robot.collection.getClutchState() == Collection.ClutchState.ENGAGED)
                robot.collection.setClutchState(Collection.ClutchState.UNENGAGED);
            else
                robot.collection.setClutchState(Collection.ClutchState.ENGAGED);

        if (gp2.isFirstLeftBumper())
            robot.collection.setFlickerState(Collection.FlickerState.HALF_UP_DOWN);
        if(gp2.isFirstLeftTrigger())
            robot.collection.setFlickerState(Collection.FlickerState.FULL_UP_DOWN);

        if (gp2.isFirstDpadLeft())
            robot.turret.changeEncoderAdjustment(Turret.turretParams.fineAdjust);
        if (gp2.isFirstDpadRight())
            robot.turret.changeEncoderAdjustment(-Turret.turretParams.fineAdjust);

        if (gp2.isFirstDpadUp())
            robot.shooter.changeVelocityAdjustment(10);
        if (gp2.isFirstDpadDown())
            robot.shooter.changeVelocityAdjustment(-10);

        if(gp2.isFirstRightBumper()) {
            robot.limelight.localization.manualPoseUpdate = true;
            robot.limelight.localization.setState(LimelightLocalization.LocalizationState.UPDATING_POSE);
        }
        if (gp2.isFirstRightTrigger()) {
            Pose2d resetPose = createPose(alliance == Alliance.RED ? redCornerResetPose : blueCornerResetPose);
            robot.drive.pinpoint().setPose(resetPose);
            robot.led.lastPinpointResetTimeMs = System.currentTimeMillis();
        }
//        if (gp2.isFirstBack())
//            robot.limelight.takePic();
    }
    private void updateDashboardField() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        robot.addRobotInfo(fieldOverlay);

        // draw goal
        fieldOverlay.setStroke("yellow");
        fieldOverlay.strokeCircle(robot.shootingSystem.goalPos.x, robot.shootingSystem.goalPos.y, 3);
        Vector2d defaultGoalPos = new Vector2d(robot.shootingSystem.goalPos.x, robot.shootingSystem.goalPos.z);
        fieldOverlay.strokeCircle(defaultGoalPos.x, defaultGoalPos.y, 3);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}