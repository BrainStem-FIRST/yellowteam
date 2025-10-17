package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Component;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.PIDController;
@Config
public class Shooter implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    private MecanumDrive drive;
    private Turret turret;
    public DcMotorEx shooterMotorFront;
    public DcMotorEx shooterMotorBack;
    public ServoImplEx hoodServo;
    public ShooterState shooterState;

    public static class Params{
        public double TORQUE_CONSTANT = 3; // 1 for far and 3 for close
        public double SHOOTER_HEIGHT = 12.89; // inches from floor to where ball ejects
        public double TARGET_HEIGHT = 48.00; // inches from floor to goal height into target
        // (the height of the front wall of the goal is 38.75 in)
        public double FLYWHEEL_RADIUS = 0.050; // meters of radius of the flywheel
        public double FLYWHEEL_TICKS_PER_REV = 288; // ticks in 1 rotation of the motor
        public double SHOOTER_POWER = 0.5;
    }

    public static Params SHOOTER_PARAMS = new Shooter.Params();

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, MecanumDrive drive, Turret turret){
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.drive = drive;
        this.turret = turret;

        shooterMotorFront = map.get(DcMotorEx.class, "shooterFront");
        shooterMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotorBack = map.get(DcMotorEx.class, "shooterBack");
        shooterMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hoodServo = map.get(ServoImplEx.class, "hood");
        hoodServo.setPwmRange(new PwmControl.PwmRange(1500, 2300));
        shooterState = ShooterState.OFF;
    }

    public double updateHoodPosition(Pose2d robotPose, Pose2d targetPose) {
        double deltaX = Math.abs(robotPose.position.x - targetPose.position.x);
        double deltaY = Math.abs(robotPose.position.y - targetPose.position.y);
        double distanceToTarget = Math.hypot(deltaX, deltaY);

        double hoodPWM = (0.0 * distanceToTarget) + 0.0; // adjust based on linear regression
        hoodServo.setPosition(hoodPWM);

        return hoodPWM;
    }


    public void setShooterPower(double power) {
        shooterMotorBack.setPower(power);
        shooterMotorFront.setPower(power);
    }

    public void updateShooterFlywheelSpeed(Pose2d robotPose, Pose2d targetPose) {
        double heightToTarget = SHOOTER_PARAMS.TARGET_HEIGHT - SHOOTER_PARAMS.SHOOTER_HEIGHT;
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double distance = Math.hypot(deltaX, deltaY);

        double hoodPWM = updateHoodPosition(robotPose, targetPose);
        double hoodAngleDeg = (-28.6 * hoodPWM) + 45.1;

        double theta = Math.toRadians(hoodAngleDeg);
        double denominator = distance * Math.tan(theta) - heightToTarget;

        // y = x*tan(θ) − (gx^2)/(2v^2cos^2(θ))

        double speedMps;
        if (denominator <= 0) {
            speedMps = 10.0; // fallback if impossible
        } else {
            speedMps = Math.sqrt(9.81 * distance * distance / (2 * Math.cos(theta) * Math.cos(theta) * denominator));
        }

        shooterMotorFront.setVelocity(speedMpsToTicksPerSec(speedMps));
        shooterMotorBack.setVelocity(speedMpsToTicksPerSec(speedMps));
    }

    private double speedMpsToTicksPerSec(double mps) {
        double circumference = 2 * Math.PI * SHOOTER_PARAMS.FLYWHEEL_RADIUS;
        double revsPerSec = mps / circumference;
        return revsPerSec * SHOOTER_PARAMS.FLYWHEEL_TICKS_PER_REV; // ticks per second
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
//        updateHoodPosition(drive.localizer.getPose(), turret.targetPose);
//        updateShooterFlywheelSpeed(drive.localizer.getPose(), turret.targetPose);
    }
    @Override
    public String test(){
        return null;
    }
}
