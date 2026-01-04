package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterState.REVERSE_FULL;

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
import org.firstinspires.ftc.teamcode.opmode.testing.PosePredictionErrorRecorder;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Parking;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.math.HeadingCorrect;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryHelper;

import java.util.List;

@Config
public class BrainSTEMTeleOp extends LinearOpMode {
    public static boolean printCollector = false, printShooter = true, printTurret = false, printLimelight = false;
    public static double firstShootTolerance = 40;

    public enum PosePredictType {
        SIMPLE,
        ADVANCED,
        CONTROL
    }
    public static PosePredictType posePredictType = PosePredictType.SIMPLE;
    public static double timeAheadToPredict = 0.075; // if this is -1, it will predict future pose next frame

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
        telemetry.addData("starting pose", startPose.position.x + ", " + startPose.position.y + " | " + startPose.heading.toDouble());
        telemetry.update();

        waitForStart();
        int framesRunning = 0;
        long startTimeNano = System.nanoTime();

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

            if (printCollector)
                robot.collection.printInfo();
            if (printLimelight)
                robot.limelight.printInfo();
            if (printTurret)
                robot.turret.printInfo();
            if (printShooter)
                robot.shooter.printInfo();

            updateDashboardField();

            // print delta time
            framesRunning++;
            double timeRunning = (System.nanoTime() - startTimeNano) * 1.0 * 1e-9;
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
        currentlyMoving = gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0;
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
            robot.collection.clutchStateTimer.reset();
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
            robot.limelight.localization.maxTranslationalError = 0;
            robot.limelight.localization.maxHeadingErrorDeg = 0;
        }
    }

    private void updateDriver2() {
        if(robot.collection.getClutchState() == Collection.ClutchState.ENGAGED) {
            if (gp2.isFirstA())
                if (robot.collection.getCollectionState() == Collection.CollectionState.INTAKE)
                    robot.collection.setCollectionState(Collection.CollectionState.OFF);
                else if (Math.abs(robot.shooter.getAvgMotorVelocity() - robot.shooter.shooterPID.getTarget()) <= firstShootTolerance)
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
            robot.turret.adjustment += Turret.turretParams.fineAdjust;
        if (gp2.isFirstDpadRight())
            robot.turret.adjustment -= Turret.turretParams.fineAdjust;

        if (gp2.isFirstDpadUp())
            robot.shooter.adjustment += 10;
        if (gp2.isFirstDpadDown())
            robot.shooter.adjustment -= 10;

        if (gp2.isFirstY()) {
            if (robot.parking.getParkState() == Parking.ParkState.EXTENDED)
                robot.turret.turretState = Turret.TurretState.PARK;
            else
                robot.parking.setParkState(Parking.ParkState.EXTENDED);
            robot.shooter.shooterState = REVERSE_FULL;
        }

        if(gp2.isFirstRightBumper()) {
            robot.limelight.localization.manualPoseUpdate = true;
            robot.limelight.localization.setState(LimelightLocalization.LocalizationState.UPDATING_POSE);
        }
        if (gp2.isFirstBack())
            robot.limelight.takePic();
    }
    private void updateDashboardField() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        robot.addRobotInfo(fieldOverlay);

        // draw goal
        fieldOverlay.setStroke("yellow");
        fieldOverlay.strokeCircle(robot.turret.targetPose.position.x, robot.turret.targetPose.position.y, 3);
        Pose2d defaultGoalPose = robot.turret.targetPose;
        fieldOverlay.strokeCircle(defaultGoalPose.position.x, defaultGoalPose.position.y, 3);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}