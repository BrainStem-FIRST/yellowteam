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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShootingMath;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryHelper;
import org.firstinspires.ftc.teamcode.utils.shootingRecording.ManualShooterSpeedRecorder;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;

import java.util.List;

@Config
@TeleOp(name="Raw subsystem test", group="Testing")
public class RawSubsystemTest extends LinearOpMode {
    public static class MiscParams {
        public int updateIntervalMs = 20;
        public boolean bulkCaching = true;
        public double intakePower = 0.99;
        public double hoodPos = 0.99;
        public boolean setHoodByAngle = true, activateLeftServo = true, activateRightServo = true;
        public boolean activateHighMotor = true, activateLowMotor = true;
        public double ballExitAngleDeg = 30;
        public double hoodInc = 0.03;
        public double flickerInPos = 0.05, flickerOutPos = 0.8;
    }
    public static MiscParams miscParams = new MiscParams();
    public static class ShooterParams {
        public boolean usePid = true;
        public double power = 0.9, targetVelTicksPerSec = 1000;
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
        ManualShooterSpeedRecorder.resetData();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        if(miscParams.bulkCaching) {
            for (LynxModule hub : allHubs)
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        DcMotorEx shooterLow = hardwareMap.get(DcMotorEx.class, "lowShoot");
        shooterLow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DcMotorEx shooterHigh = hardwareMap.get(DcMotorEx.class, "highShoot");
        shooterHigh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        PIDController shooterPid = new PIDController(sParams.kP, sParams.kI, sParams.kD);

        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ServoImplEx hoodLeft = hardwareMap.get(ServoImplEx.class, "hoodLeft");
        hoodLeft.setPwmRange(new PwmControl.PwmRange(Shooter.hoodParams.downPWM, Shooter.hoodParams.upPWM));
        ServoImplEx hoodRight = hardwareMap.get(ServoImplEx.class, "hoodRight");
        hoodRight.setPwmRange(new PwmControl.PwmRange(Shooter.hoodParams.downPWM, Shooter.hoodParams.upPWM));

        shooterLow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterHigh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        DcMotorEx collectorMotor = hardwareMap.get(DcMotorEx.class, "intake");
        collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ServoImplEx clutchRight = hardwareMap.get(ServoImplEx.class, "clutchRight");
        clutchRight.setPwmRange(new PwmControl.PwmRange(1450, 2000));

        ServoImplEx clutchLeft = hardwareMap.get(ServoImplEx.class, "clutchLeft");
        clutchLeft.setPwmRange(new PwmControl.PwmRange(1450, 2000));

        ServoImplEx flickerLeft = hardwareMap.get(ServoImplEx.class, "flickerLeft");
        flickerLeft.setPwmRange(new PwmControl.PwmRange(Collection.params.flickerLeftMinPwm, Collection.params.flickerLeftMaxPwm));
        ServoImplEx flickerRight = hardwareMap.get(ServoImplEx.class, "flickerRight");
        flickerRight.setPwmRange(new PwmControl.PwmRange(Collection.params.flickerRightMinPwm, Collection.params.flickerRightMaxPwm));

        GamepadTracker g1 = new GamepadTracker(gamepad1);

        boolean shooting = false;
        double maxShooterVel = 0; // max vel before ball is shot

        ElapsedTime dtTimer = new ElapsedTime();

        waitForStart();
        while(opModeIsActive()) {
            dtTimer.reset();

            if(miscParams.bulkCaching) {
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

            turretMotor.setPower(0);

            if(g1.isFirstLeftBumper()) {
                if(flickerLeft.getPosition() < 0.5) {
                    flickerLeft.setPosition(miscParams.flickerOutPos);
                    flickerRight.setPosition(miscParams.flickerOutPos);
                }
                else {
                    flickerLeft.setPosition(miscParams.flickerInPos);
                    flickerRight.setPosition(miscParams.flickerInPos);
                }
            }

            if(g1.isFirstY()) {
                shooting = !shooting;
                shooterPid.reset();
            }
            if(shooting) {
                if(sParams.usePid) {
                    double pidPower = shooterPid.updateWithError(sParams.targetVelTicksPerSec - Math.abs(shooterHigh.getVelocity()));
                    double feedForward = sParams.kF * sParams.targetVelTicksPerSec;
                    double power = pidPower + feedForward;
                    if(miscParams.activateLowMotor)
                        shooterLow.setPower(power);
                    if(miscParams.activateHighMotor)
                        shooterHigh.setPower(power * sParams.dirFlip);
                }
                else {
                    if (miscParams.activateLowMotor)
                        shooterLow.setPower(sParams.power);
                    if (miscParams.activateHighMotor)
                        shooterHigh.setPower(sParams.power * sParams.dirFlip);
                }
            }
            else {
                shooterLow.setPower(0);
                shooterHigh.setPower(0);
            }

            if(g1.isFirstDpadUp())
                miscParams.hoodPos += miscParams.hoodInc;
            else if(g1.isFirstDpadDown())
                miscParams.hoodPos -= miscParams.hoodInc;

            if(g1.isFirstA())
                if(Math.abs(collectorMotor.getPower()) > 0.5)
                    collectorMotor.setPower(0);
                else
                    collectorMotor.setPower(miscParams.intakePower);

            else if(g1.isFirstB()) {
                if (clutchLeft.getPosition() > 0.5) {
                    clutchLeft.setPosition(Collection.params.ENGAGED_POS);
                    clutchRight.setPosition(Collection.params.ENGAGED_POS);
                } else {
                    clutchLeft.setPosition(Collection.params.DISENGAGED_POS);
                    clutchRight.setPosition(Collection.params.DISENGAGED_POS);

                }
            }
            if (miscParams.setHoodByAngle)
                miscParams.hoodPos = ShootingMath.calculateHoodServoPosition(Math.toRadians(miscParams.ballExitAngleDeg));
            if (miscParams.activateLeftServo)
                hoodLeft.setPosition(miscParams.hoodPos);
            if (miscParams.activateRightServo)
                hoodRight.setPosition(miscParams.hoodPos);

            if (g1.isFirstLeftBumper()) {
                shotRecordNum = 0;
            }
            if (gamepad1.left_bumper) {
                long currentTime = System.nanoTime();
                if ((currentTime - lastRecordTimeNano) * 1e-6 > miscParams.updateIntervalMs) {
                    lastRecordTimeNano = currentTime;

                    ManualShooterSpeedRecorder.data[ManualShooterSpeedRecorder.getCurrentShot()][shotRecordNum][1] = shooterPid.getTarget();
                    ManualShooterSpeedRecorder.data[ManualShooterSpeedRecorder.getCurrentShot()][shotRecordNum][2] = Math.abs(shooterLow.getVelocity());
                    ManualShooterSpeedRecorder.data[ManualShooterSpeedRecorder.getCurrentShot()][shotRecordNum][3] = shooterLow.getPower();
                    shotRecordNum++;
                }
            }
            if (gamepad1.leftBumperWasReleased())
                ManualShooterSpeedRecorder.incrementCurrentShot();


//            telemetry.addData("bulk caching", bulkCaching);
//            telemetry.addData("using pid", sParams.usePid);
//            telemetry.addData("dt", dtTimer.seconds());
//            telemetry.addData("fps", 1/dtTimer.seconds());
            telemetry.addLine();

            double lowShootC = shooterLow.getCurrent(CurrentUnit.AMPS);
            double rbC = drive.rightBack.getCurrent(CurrentUnit.AMPS);
            double rfC = drive.rightFront.getCurrent(CurrentUnit.AMPS);
            double intakeC = collectorMotor.getCurrent(CurrentUnit.AMPS);
            double controlC = lowShootC + rbC + rfC + intakeC;
            telemetry.addData("flicker left position", flickerLeft.getPosition());
            telemetry.addData("flicker right position", flickerRight.getPosition());
            telemetry.addData("lowShootC", lowShootC);
            telemetry.addData("rbC", rbC);
            telemetry.addData("rfC", rfC);
            telemetry.addData("intakeC", intakeC);
            telemetry.addData("control hub current", controlC);

            double highShootC = shooterHigh.getCurrent(CurrentUnit.AMPS);
            double lbC = drive.leftBack.getCurrent(CurrentUnit.AMPS);
            double lfC = drive.leftFront.getCurrent(CurrentUnit.AMPS);
            double turretC = turretMotor.getCurrent(CurrentUnit.AMPS);
            double expansC = highShootC + lbC + lfC + turretC;
            telemetry.addData("shootHighC", highShootC);
            telemetry.addData("lbC", lbC);
            telemetry.addData("lfC", lfC);
            telemetry.addData("turretC", turretC);
            telemetry.addData("expansion current", expansC);
            telemetry.addData("total c", controlC + expansC);
            telemetry.addLine();
            telemetry.addData("1 power", shooterLow.getPower());
            telemetry.addData("2 power", shooterHigh.getPower());
            telemetry.addData("1 encoder", shooterLow.getCurrentPosition());
            telemetry.addData("2 encoder", shooterHigh.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("shooter current", lowShootC + highShootC);

            telemetry.addLine();
            telemetry.addData("ball exit height meters at target hood angle", ShootingMath.approximateExitHeightMeters(true));
            telemetry.addData("ball exit vel (m/s)", ShootingMath.ticksPerSecToExitSpeedMps(Math.abs(shooterHigh.getVelocity()), ShootingMath.shooterSystemParams.powerLossCoefficient));
            telemetry.addData("1 vel (ticks/s)", shooterLow.getVelocity());
            telemetry.addData("2 vel (ticks/s)", shooterHigh.getVelocity());
            telemetry.addData("target", sParams.targetVelTicksPerSec);
            telemetry.addLine();
            telemetry.addData("setting hood by angle", miscParams.setHoodByAngle);
            telemetry.addData("target hood pos", miscParams.hoodPos);
            telemetry.addData("hoodL pos", hoodLeft.getPosition());
            telemetry.addData("hoodR pos", hoodRight.getPosition());
            telemetry.addLine();
            telemetry.addData("intake power", collectorMotor.getPower());
            telemetry.addData("left clutch pos", clutchLeft.getPosition());
            telemetry.addData("right clutch pos", clutchRight.getPosition());

            updateDashboardField(turretMotor.getCurrentPosition(), Math.toRadians(miscParams.ballExitAngleDeg));
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
