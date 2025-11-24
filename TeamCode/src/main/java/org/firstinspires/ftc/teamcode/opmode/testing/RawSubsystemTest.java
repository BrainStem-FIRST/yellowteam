package org.firstinspires.ftc.teamcode.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMath;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryHelper;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;

import java.util.List;

@Config
@TeleOp(name="Raw subsystem test")
public class RawSubsystemTest extends LinearOpMode {
    public static int updateIntervalMs = 20;
    public static boolean bulkCaching = true;
    public static double intakePower = 0.99;
    public static double hoodPos = 0.87;
    public static boolean setHoodByAngle = true, activateLeftServo = true, activateRightServo = true;
    public static boolean activateHighMotor = true, activateLowMotor = true;
    public static double targetHoodAngleDeg = 30;
    public static double hoodInc = 0.03;
    public static class ShooterParams {
        public boolean usePid = true;
        public double power = 0.9, targetVelTicksPerSec = 1700;
        public int dirFlip = -1;
        public double kP = 0.005;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.00053;
    }
    public static ShooterParams sParams = new ShooterParams();
    private int shotRecordNum = 0;
    private long lastRecordTimeNano;

    @Override
    public void runOpMode() throws InterruptedException {
        ShooterSpeedRecorder.resetData();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        if(bulkCaching) {
            for (LynxModule hub : allHubs)
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "lowShoot");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "highShoot");
        PIDController shooterPid = new PIDController(sParams.kP, sParams.kI, sParams.kD);

        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ServoImplEx hoodLeft = hardwareMap.get(ServoImplEx.class, "hoodLeft");
        hoodLeft.setPwmRange(new PwmControl.PwmRange(Shooter.HOOD_PARAMS.downPWM, Shooter.HOOD_PARAMS.upPWM));
        ServoImplEx hoodRight = hardwareMap.get(ServoImplEx.class, "hoodRight");
        hoodRight.setPwmRange(new PwmControl.PwmRange(Shooter.HOOD_PARAMS.downPWM, Shooter.HOOD_PARAMS.upPWM));

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        DcMotorEx collectorMotor = hardwareMap.get(DcMotorEx.class, "intake");
        collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ServoImplEx clutchRight = hardwareMap.get(ServoImplEx.class, "clutchRight");
        clutchRight.setPwmRange(new PwmControl.PwmRange(1450, 2000));

        ServoImplEx clutchLeft = hardwareMap.get(ServoImplEx.class, "clutchLeft");
        clutchLeft.setPwmRange(new PwmControl.PwmRange(1450, 2000));

        GamepadTracker g1 = new GamepadTracker(gamepad1);

        boolean shooting = false;
        double maxShooterVel = 0; // max vel before ball is shot

        ElapsedTime dtTimer = new ElapsedTime();

        waitForStart();
        while(opModeIsActive()) {
            dtTimer.reset();
            if(bulkCaching) {
                for (LynxModule hub : allHubs)
                    hub.clearBulkCache();
            }

            g1.update();

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            if(g1.isFirstY()) {
                shooting = !shooting;
                shooterPid.reset();
            }
            if(shooting) {
                if(sParams.usePid) {
                    double pidPower = shooterPid.updateWithError(sParams.targetVelTicksPerSec - shooter2.getVelocity());
                    double feedForward = sParams.kF * sParams.targetVelTicksPerSec;
                    double power = pidPower + feedForward;
                    if(activateLowMotor)
                        shooter1.setPower(power);
                    if(activateHighMotor)
                        shooter2.setPower(power * sParams.dirFlip);
                }
                else {
                    if(activateLowMotor)
                        shooter1.setPower(sParams.power);
                    if(activateHighMotor)
                        shooter2.setPower(sParams.power * sParams.dirFlip);
                }
            }
            else {
                shooter1.setPower(0);
                shooter2.setPower(0);
            }

            if(g1.isFirstDpadUp())
                hoodPos += hoodInc;
            else if(g1.isFirstDpadDown())
                hoodPos -= hoodInc;

            if(g1.isFirstA())
                if(Math.abs(collectorMotor.getPower()) > 0.5)
                    collectorMotor.setPower(0);
                else
                    collectorMotor.setPower(intakePower);

            else if(g1.isFirstB()) {
                if (clutchLeft.getPosition() > 0.5) {
                    clutchLeft.setPosition(Collection.COLLECTOR_PARAMS.ENGAGED_POS);
                    clutchRight.setPosition(Collection.COLLECTOR_PARAMS.ENGAGED_POS);
                } else {
                    clutchLeft.setPosition(Collection.COLLECTOR_PARAMS.DISENGAGED_POS);
                    clutchRight.setPosition(Collection.COLLECTOR_PARAMS.DISENGAGED_POS);

                }
            }
            if (setHoodByAngle)
                hoodPos = ShootingMath.calculateHoodServoPosition(Math.toRadians(targetHoodAngleDeg), telemetry);
            if (activateLeftServo)
                hoodLeft.setPosition(hoodPos);
            if (activateRightServo)
                hoodRight.setPosition(hoodPos);

            if (g1.isFirstLeftBumper()) {
                shotRecordNum = 0;
            }
            if (gamepad1.left_bumper) {
                long currentTime = System.nanoTime();
                if ((currentTime - lastRecordTimeNano) * 1e-6 > updateIntervalMs) {
                    lastRecordTimeNano = currentTime;

                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][shotRecordNum][1] = shooterPid.getTarget();
                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][shotRecordNum][2] = Math.abs(shooter1.getVelocity());
                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][shotRecordNum][3] = shooter1.getPower();
                    shotRecordNum++;
                }
            }
            if (gamepad1.leftBumperWasReleased())
                ShooterSpeedRecorder.incrementCurrentShot();

//            if(g1.isFirstLeftBumper())
//                shootVelocities.clear();
//            if(shooterVelTicks < sParams.targetVelTicksPerSec - sParams.shootVelDropThreshold && shooterVelocityTimer.seconds() > sParams.shootResetTime) {
//                shooterVelocityTimer.reset();
//                shootVelocities.add(maxShooterVel);
//            }


            telemetry.addData("bulk caching", bulkCaching);
            telemetry.addData("using pid", sParams.usePid);
            telemetry.addData("dt", dtTimer.seconds());
            telemetry.addData("fps", 1/dtTimer.seconds());
            telemetry.addLine();
            telemetry.addData("1 power", shooter1.getPower());
            telemetry.addData("2 power", shooter2.getPower());
            telemetry.addData("target", sParams.targetVelTicksPerSec);

            telemetry.addLine();
            telemetry.addData("ball exit height meters at target hood angle", ShootingMath.calculateExitHeightMeters(Math.toRadians(targetHoodAngleDeg)));
            telemetry.addData("1 vel (ticks/s)", shooter1.getVelocity());
            telemetry.addData("2 vel (ticks/s)", shooter2.getVelocity());
            telemetry.addData("ball exit vel (m/s)", ShootingMath.ticksPerSecToExitSpeedMps(Math.abs(shooter2.getVelocity())));
            telemetry.addData("1 encoder", shooter1.getCurrentPosition());
            telemetry.addData("2 encoder", shooter2.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("setting hood by angle", setHoodByAngle);
            telemetry.addData("target hood pos", hoodPos);
            telemetry.addData("hoodL pos", hoodLeft.getPosition());
            telemetry.addData("hoodR pos", hoodRight.getPosition());
            telemetry.addLine();
            telemetry.addData("intake power", collectorMotor.getPower());
            telemetry.addData("left clutch pos", clutchLeft.getPosition());
            telemetry.addData("right clutch pos", clutchRight.getPosition());

            updateDashboardField(turretMotor.getCurrentPosition(), Math.toRadians(targetHoodAngleDeg));
            telemetry.update();
        }
    }
    private void updateDashboardField(int turretEncoder, double hoodAngleRad) {

        double shooterCombinedRadiusInches = (ShootingMath.shooterSystemParams.flywheelRadiusMeters + ShootingMath.shooterSystemParams.ballRadiusMeters) / 0.0254;
        double offsetFromTurretInches = ShootingMath.shooterSystemParams.flywheelOffsetFromTurretInches - Math.cos(hoodAngleRad) * shooterCombinedRadiusInches;
        telemetry.addData("offset from turret inches", offsetFromTurretInches);
        Pose2d robotPose = new Pose2d(0, 0, 0);
        Pose2d turretPose = Turret.getTurretPose(robotPose, turretEncoder);
        Vector2d exitPosition = ShootingMath.calculateExitPositionInches(robotPose, turretEncoder, hoodAngleRad);
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        TelemetryHelper.addRobotPoseToCanvas(fieldOverlay, robotPose, turretPose, new Pose2d(exitPosition.x, exitPosition.y, turretPose.heading.toDouble()));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

//       TelemetryHelper.sendRobotPoses(robotPose, robot.turret.getTurretPose(robotPose));
    }
}
