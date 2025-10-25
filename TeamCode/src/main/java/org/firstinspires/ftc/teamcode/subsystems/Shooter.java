package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Component;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.PIDController;

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
        public double FLYWHEEL_TICKS_PER_REV = 28; // ticks in 1 rotation of the motor
        public double SHOOTER_POWER = 0.5;
        public double HOOD_INCREMENT = 0.1;
        public double CLOSE_SHOOTER_POWER = 0.7;
        public double FAR_SHOOTER_POWER = 0.9;
        public double ZONE_THRESHOLD = 100;
        public double B_VALUE = 64.61487;
        public double TARGET_VELOCITY = 1300;

        public double kP = 0.05;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.00047;
    }

    public static Params SHOOTER_PARAMS = new Shooter.Params();
    private PIDController shooterPID;

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

        shooterPID = new PIDController(SHOOTER_PARAMS.kP, SHOOTER_PARAMS.kI, SHOOTER_PARAMS.kD);

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

    public void setShooterVelocityPID(double targetVelocityTicksPerSec) {
        shooterPID.setTarget(targetVelocityTicksPerSec);

        double currentVelocity = (shooterMotorLow.getVelocity() + shooterMotorHigh.getVelocity()) / 2.0;
        double pidOutput = -shooterPID.update(currentVelocity);

        double feedForward = SHOOTER_PARAMS.kF * targetVelocityTicksPerSec;
        double totalPower = pidOutput + feedForward;

        totalPower = Range.clip(totalPower, -1.0, 1.0);

        setShooterPower(totalPower);

        telemetry.addData("Shooter Target Vel", targetVelocityTicksPerSec);
        telemetry.addData("Shooter Current Vel", currentVelocity);
        telemetry.addData("Time", System.currentTimeMillis());
        telemetry.addData("Shooter PID Output", pidOutput);
        telemetry.addData("Shooter FeedForward", feedForward);
        telemetry.addData("Shooter Total Power", totalPower);
    }

    public enum ShootingZone {
        CLOSE, FAR
    }

    /*/
              +y
              ↑
   (-72,72)   |  (72,72)
              |
--------------+--------------> +x
              |
   (-72,-72)  |  (72,-72)
              |
    /*/

    public ShootingZone getShootingZone(Pose2d robotPose, Pose2d targetPose) {
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double distance = Math.hypot(deltaX, deltaY);

        telemetry.addData("Dist to Shooter", distance);

        if (distance <= SHOOTER_PARAMS.ZONE_THRESHOLD)
            return ShootingZone.CLOSE;
        else
            return ShootingZone.FAR;
    }

    public void setFlywheelSpeedByZone(ShootingZone zone) {
        double targetPower = (zone == ShootingZone.CLOSE)
                ? SHOOTER_PARAMS.CLOSE_SHOOTER_POWER
                : SHOOTER_PARAMS.FAR_SHOOTER_POWER;

        setShooterPower(targetPower);
    }

    public double calculateHoodAngle(Pose2d robotPose, Pose2d targetPose) {
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double distance = Math.hypot(deltaX, deltaY);

        double actualVelocityMps = ticksPerSecToMps((shooterMotorLow.getVelocity() + shooterMotorHigh.getVelocity()) / 2.0);
//        telemetry.addData("MPS", actualVelocityMps);
        double heightToTarget = (SHOOTER_PARAMS.TARGET_HEIGHT - SHOOTER_PARAMS.SHOOTER_HEIGHT);

        // Physics formula rearranged for angle: tan(θ) = (v² ± √(v⁴ - g(gx² + 2yh²))) / (gx)
        double g = 9.81;
        double x = distance * 0.0254; // convert inches to meters
        double y = heightToTarget * 0.0254; // convert inches to meters

        double v = actualVelocityMps;

        double discriminant = v*v*v*v - g*(g*x*x + 2*y*v*v);
        if (discriminant <= 0)
            return Math.toRadians(40);

        double theta = Math.atan( (v*v - Math.sqrt(discriminant)) / (g * x) );

        telemetry.addData("HOOD ANGLE", Math.toDegrees(theta));

        return theta;
    }

    public void setHoodFromAngle(double angleRadians) {
        double angleDeg = Math.toDegrees(angleRadians);
        angleDeg = Math.max(16.5, Math.min(45.1, angleDeg));
        double servoPos = (-0.03497 * angleDeg) + 1.578; //angle(16.5-45.1) conversion to servo pos(1-0)

        telemetry.addData("HOOD SERVO POS", servoPos);

        setHoodPosition(servoPos);
    }

    public void updateShooterSystem(Pose2d robotPose, Pose2d targetPose) {
//        ShootingZone zone = getShootingZone(robotPose, targetPose);
//        setFlywheelSpeedByZone(zone);

//        double deltaX = targetPose.position.x - robotPose.position.x;
//        double deltaY = targetPose.position.y - robotPose.position.y;
//        double distance = Math.hypot(deltaX, deltaY);

//        setShooterPower((0.112398 * distance + SHOOTER_PARAMS.B_VALUE)/100); //linear distance to power correlation
//        setShooterPower((SHOOTER_PARAMS.B_VALUE * Math.pow(1.0016, distance))/100 + 0.1); //exponential distance to power correlation
//        telemetry2.addData("CALC POW", 63.61487 * Math.pow(1.0016, distance));

        setShooterVelocityPID(SHOOTER_PARAMS.TARGET_VELOCITY);
        calculateHoodAngle(robotPose, targetPose);
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
        OFF, SHOOT, UPDATE, VELOCITY_CONTROL
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

            case UPDATE:
//                setShooterPower(SHOOTER_PARAMS.SHOOTER_POWER);
                updateShooterSystem(drive.localizer.getPose(), turret.targetPose);
                break;

            case SHOOT:
                setShooterPower(SHOOTER_PARAMS.SHOOTER_POWER);
                break;

            case VELOCITY_CONTROL:
                setShooterVelocityPID(SHOOTER_PARAMS.TARGET_VELOCITY);
                break;
        }

//        updateShooterSystem(drive.localizer.getPose(), turret.targetPose);
        telemetry.addData("SHOOTER CURRENT", shooterMotorHigh.getCurrent(CurrentUnit.MILLIAMPS));
//        telemetry.addData("Shooter Ticks", shooterMotorHigh.getCurrentPosition());
//        telemetry.addData("Shooter Vel", (shooterMotorLow.getVelocity() + shooterMotorHigh.getVelocity()) / 2.0);
//        telemetry.addData("Shooting Zone", getShootingZone(drive.localizer.getPose(), turret.targetPose).toString());
    }
    @Override
    public String test(){
        return null;
    }
}
