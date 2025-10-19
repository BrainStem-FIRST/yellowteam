package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
public class Shooter implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    private MecanumDrive drive;
    private Turret turret;
    public DcMotorEx shooterMotorLow;
    public DcMotorEx shooterMotorHigh;
    public ServoImplEx hoodLeftServo;
    public ServoImplEx hoodRightServo;
    public ShooterState shooterState;

    public static class Params{
        public double SHOOTER_HEIGHT = 12.89; // inches from floor to where ball ejects
        public double TARGET_HEIGHT = 48.00; // inches from floor to goal height into target
        // (the height of the front wall of the goal is 38.75 in)
        public double FLYWHEEL_RADIUS = 0.050; // meters of radius of the flywheel
        public double FLYWHEEL_TICKS_PER_REV = 288; // ticks in 1 rotation of the motor
        public double SHOOTER_POWER = 1.0;
        public double HOOD_INCREMENT = 0.1;
        public double CLOSE_SHOOTER_VELOCITY = 1;
        public double FAR_SHOOTER_VELOCITY = 1;
        public double ZONE_THRESHOLD = 60;
    }

    public static Params SHOOTER_PARAMS = new Shooter.Params();

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, MecanumDrive drive, Turret turret){
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.drive = drive;
        this.turret = turret;

        shooterMotorLow = map.get(DcMotorEx.class, "lowShoot");
        shooterMotorLow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotorHigh = map.get(DcMotorEx.class, "highShoot");
        shooterMotorHigh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorHigh.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodLeftServo = map.get(ServoImplEx.class, "hoodLeft");
        hoodLeftServo.setPwmRange(new PwmControl.PwmRange(1000, 1800));

        hoodRightServo = map.get(ServoImplEx.class, "hoodRight");
        hoodRightServo.setPwmRange(new PwmControl.PwmRange(1000, 1800));

        shooterState = ShooterState.OFF;
    }

    public void setShooterPower(double power) {
        shooterMotorLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotorHigh.setPower(power);
        shooterMotorLow.setPower(power);
    }

    public void setShooterVelocity(double targetVelocityTicksPerSec) {
        shooterMotorLow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorHigh.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterMotorLow.setVelocity(targetVelocityTicksPerSec);
        shooterMotorHigh.setVelocity(targetVelocityTicksPerSec);
    }

    public enum ShootingZone {
        CLOSE, FAR
    }

    public ShootingZone getShootingZone(Pose2d robotPose, Pose2d targetPose) {
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double distance = Math.hypot(deltaX, deltaY);

        if (distance <= SHOOTER_PARAMS.ZONE_THRESHOLD)
            return ShootingZone.CLOSE;
        else
            return ShootingZone.FAR;
    }

    public void setFlywheelSpeedByZone(ShootingZone zone) {
        double targetVelocity = (zone == ShootingZone.CLOSE)
                ? SHOOTER_PARAMS.CLOSE_SHOOTER_VELOCITY
                : SHOOTER_PARAMS.FAR_SHOOTER_VELOCITY;

        setShooterVelocity(targetVelocity);
    }

    public double calculateHoodAngle(Pose2d robotPose, Pose2d targetPose) {
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double distance = Math.hypot(deltaX, deltaY);

        double actualVelocityMps = ticksPerSecToMps((shooterMotorLow.getVelocity() + shooterMotorHigh.getVelocity()) / 2.0);

        double heightToTarget = SHOOTER_PARAMS.TARGET_HEIGHT - SHOOTER_PARAMS.SHOOTER_HEIGHT;

        // Physics formula rearranged for angle: tan(θ) = (v² ± √(v⁴ - g(gx² + 2yh²))) / (gx)
        double g = 9.81;
        double term = Math.pow(actualVelocityMps, 4)
                - g * (g * Math.pow(distance * 0.0254, 2) + 2 * heightToTarget * 0.0254 * Math.pow(actualVelocityMps, 2));

        if (term <= 0)
            return Math.toRadians(40); //safe angle

        double numerator = Math.pow(actualVelocityMps, 2) - Math.sqrt(term);
        double denominator = g * distance * 0.0254;

        return Math.atan(numerator / denominator); //theta
    }

    public void setHoodFromAngle(double angleRadians) {
        double angleDeg = Math.toDegrees(angleRadians);
        angleDeg = Math.max(16.5, Math.min(45.1, angleDeg));
        double servoPos = (-0.03497 * angleDeg) + 1.578; //angle(16.5-45.1) conversion to servo pos(0-1)

        hoodLeftServo.setPosition(servoPos);
        hoodRightServo.setPosition(servoPos);
    }

    public void updateShooterSystem(Pose2d robotPose, Pose2d targetPose) {
        ShootingZone zone = getShootingZone(robotPose, targetPose);
        setFlywheelSpeedByZone(zone);

        double hoodAngle = calculateHoodAngle(robotPose, targetPose);
        setHoodFromAngle(hoodAngle);
    }

    public double ticksPerSecToMps(double ticksPerSec) {
        double wheelCircumference = 2 * Math.PI * SHOOTER_PARAMS.FLYWHEEL_RADIUS;
        return (ticksPerSec / SHOOTER_PARAMS.FLYWHEEL_TICKS_PER_REV) * wheelCircumference;
    }

    public void setHoodPosition(double position) {
        hoodLeftServo.setPosition(position);
        hoodRightServo.setPosition(position);
    }

    public enum ShooterState {
        OFF, SHOOT
    }

    @Override
    public void reset() {
    }

    @Override
    public void update(){
        switch (shooterState) {
            case OFF:
                setShooterPower(0);
                break;

            case SHOOT:
                setShooterPower(SHOOTER_PARAMS.SHOOTER_POWER);
                break;
        }

//        updateShooterSystem(drive.localizer.getPose(), turret.targetPose);
        telemetry.addData("Shooting Zone", getShootingZone(drive.localizer.getPose(), turret.targetPose).toString());
    }
    @Override
    public String test(){
        return null;
    }
}
