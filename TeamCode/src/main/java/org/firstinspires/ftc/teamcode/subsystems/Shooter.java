package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.testing.ShooterSpeedRecorder;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;

import java.util.Collections;
import java.util.Set;

@Config
public class Shooter extends Component {
    public static class ShooterParams {
        public double FLYWHEEL_HEIGHT_METERS = 0.2413;
        public double TARGET_HEIGHT_INCHES = 48.00; // inches from floor to goal height into target
        // (the height of the front wall of the goal is 38.75 in)

        public double WALL_OFFSET_INCHES = 30;
        public double FLYWHEEL_OFFSET_FROM_TURRET_INCHES = 2.4783465;
        public double FLYWHEEL_RADIUS_METERS = 0.050; // meters of radius of the flywheel
        public double BALL_RADIUS_METERS = 0.064;
        public double GRIP_COEFFICIENT = 1;// actual exit velocity / theoretical exit velocity
        public double SHOOTER_MOTOR_TICKS_PER_REV = 28;

        // pulley ratio is 30:22
        public double FLYWHEEL_TICKS_PER_REV = SHOOTER_MOTOR_TICKS_PER_REV * 30 / 22; // ticks in 1 rotation of flywheel
        public double B_CLOSE_VALUE = 800.29968;
        public double SLOPE_CLOSE_VALUE = 4.53882;
        public double FAR_TARGET_VEL_TICKS = 1525;
        public double RELATIVE_VELOCITY_CORRECTION = 1;

        public double kP = 0.005;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.00056;
    }
    public static class HoodParams {
        public double restingDistanceMm = 82;
        public double hoodPivotAngleOffsetFromHoodExitAngleDeg = 7.8;
        public double downPWM = 900, upPWM = 2065;
        public double servoRangeMm = 30;
        public static double minAngleDeg = 20, maxAngleDeg = 50;
    }
    public static ShooterParams SHOOTER_PARAMS = new ShooterParams();
    public static HoodParams HOOD_PARAMS = new HoodParams();
    public static boolean ENABLE_TESTING = false;
    public static boolean useRelativeVelocity = false;

    public enum ShooterState {
        OFF, UPDATE, FIXED_VELOCITY_IN_AUTO
    }
    public enum ShootingZone {
        CLOSE, FAR
    }

    /*
              +y
              ↑
   (-72,72)   |  (72,72)
              |
--------------+--------------> +x
              |
   (-72,-72)  |  (72,-72)
              |
    */
    public static double testingShootPower = 0.99, testingBallExitAngleRad = 0.7;
    public DcMotorEx shooterMotorLow;
    public DcMotorEx shooterMotorHigh;
    public ServoImplEx hoodLeftServo;
    public ServoImplEx hoodRightServo;
    public ShooterState shooterState;

    private final PIDController shooterPID;
    private double ballExitAngleRad;
    private double hoodServoPos;
    public double adjustment;
    private final ElapsedTime recordTimer;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        shooterMotorLow = hardwareMap.get(DcMotorEx.class, "lowShoot");
        shooterMotorLow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLow.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotorHigh = hardwareMap.get(DcMotorEx.class, "highShoot");
        shooterMotorHigh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorHigh.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodLeftServo = hardwareMap.get(ServoImplEx.class, "hoodLeft");
        hoodLeftServo.setPwmRange(new PwmControl.PwmRange(HOOD_PARAMS.downPWM, HOOD_PARAMS.upPWM));

        hoodRightServo = hardwareMap.get(ServoImplEx.class, "hoodRight");
        hoodRightServo.setPwmRange(new PwmControl.PwmRange(HOOD_PARAMS.downPWM, HOOD_PARAMS.upPWM));

        shooterPID = new PIDController(SHOOTER_PARAMS.kP, SHOOTER_PARAMS.kI, SHOOTER_PARAMS.kD);

        shooterState = ShooterState.OFF;
        recordTimer = new ElapsedTime();
    }

    public double getBallExitAngleRad() {
        return ballExitAngleRad;
    }
    public void setShooterPower(double power) {
        shooterMotorHigh.setPower(power);
        shooterMotorLow.setPower(power);
    }
    public void setShooterVelocityPID(double targetVelocityTicksPerSec) {
        shooterPID.setTarget(targetVelocityTicksPerSec + adjustment);

        double currentVelocity = getAvgMotorVelocity();
        double pidOutput = -shooterPID.update(currentVelocity);
        telemetry.addData("pid output", pidOutput);
        double feedForward = SHOOTER_PARAMS.kF * targetVelocityTicksPerSec;
        double totalPower = pidOutput + feedForward;

        totalPower = Math.abs(Range.clip(totalPower, -0.99, 0.99));
        telemetry.addData("total power", totalPower);

        setShooterPower(totalPower);
    }
    public double calculateBallExitAngleRad(Vector2d ballExitPosition, Pose2d targetPose, double distance) {
        double ballExitSpeedMps = ticksPerSecToExitSpeedMps(getAvgMotorVelocity());

        double exitHeightMeters = getExitHeightMeters(ballExitAngleRad);
        double heightToTargetMeters = (SHOOTER_PARAMS.TARGET_HEIGHT_INCHES * 0.0254 - exitHeightMeters);

        // Physics formula rearranged for angle: tan(θ) = (v² ± √(v⁴ - g(gx² + 2yv²))) / (gx)
        double g = 9.81;
        double x = distance * 0.0254; // convert inches to meters
        double y = heightToTargetMeters; // convert inches to meters

        double robotVelToGoalMps = computeVelocityTowardGoalMps(ballExitPosition, targetPose);
        double v = ballExitSpeedMps + robotVelToGoalMps * SHOOTER_PARAMS.RELATIVE_VELOCITY_CORRECTION;
        double discriminant = v*v*v*v - g*(g*x*x + 2*y*v*v);
        if (discriminant <= 0)
            return Math.toRadians(40);

        double tanTheta = (v*v - Math.sqrt(discriminant)) / (g * x);
        return Math.atan(tanTheta);
    }
    public double computeVelocityTowardGoalMps(Vector2d ballExitPosition, Pose2d targetPose) {
        double dx = targetPose.position.x - ballExitPosition.x;
        double dy = targetPose.position.y - ballExitPosition.y;

        double distanceFromGoal = Math.hypot(dx, dy);
        double ux = dx / distanceFromGoal;
        double uy = dy / distanceFromGoal;

        OdoInfo vel = robot.drive.pinpoint().getMostRecentVelocity();
        double vx = vel.x;
        double vy = vel.y;

        return -(vx*ux + vy*uy) * 0.0254;
    }

    public void updateShooterSystem(Vector2d ballExitPosition, Pose2d targetPose) {
        double deltaX = targetPose.position.x - ballExitPosition.x;
        double deltaY = targetPose.position.y - ballExitPosition.y;
        double inchesFromGoal = Math.hypot(deltaX, deltaY) + SHOOTER_PARAMS.WALL_OFFSET_INCHES;

        double velocityTicks;

        if (ballExitPosition.x < 50) {
            double absoluteExitSpeedTicksPerSec = (SHOOTER_PARAMS.SLOPE_CLOSE_VALUE * inchesFromGoal) + SHOOTER_PARAMS.B_CLOSE_VALUE;
            if (useRelativeVelocity) {
                double absoluteExitSpeedMps = ticksPerSecToExitSpeedMps(absoluteExitSpeedTicksPerSec);
                double exitPositionSpeedTowardsGoalMps = computeVelocityTowardGoalMps(ballExitPosition, targetPose); // SHOULD return positive value
                double relativeExitSpeedMps = absoluteExitSpeedMps - (exitPositionSpeedTowardsGoalMps * SHOOTER_PARAMS.RELATIVE_VELOCITY_CORRECTION);
                telemetry.addData("ADJUSTED POWER", mpsToTicksPerSec(relativeExitSpeedMps));
                telemetry.addData("BALL EXIT MPS", exitPositionSpeedTowardsGoalMps);

                velocityTicks = mpsToTicksPerSec(relativeExitSpeedMps);
            }
            else
                velocityTicks = absoluteExitSpeedTicksPerSec;
        }
        else
            velocityTicks = SHOOTER_PARAMS.FAR_TARGET_VEL_TICKS;

        setShooterVelocityPID(velocityTicks);

        if (ENABLE_TESTING) {
            hoodServoPos = getHoodServoPosition(testingBallExitAngleRad, telemetry);
            setHoodPosition(hoodServoPos);
        }
        else {
            ballExitAngleRad = calculateBallExitAngleRad(ballExitPosition, targetPose, inchesFromGoal);
            if (ballExitPosition.x < 50) {
                hoodServoPos = getHoodServoPosition(ballExitAngleRad, telemetry);
                setHoodPosition(hoodServoPos);
            }
            else
                setHoodPosition(1);
        }
    }

    public static double ticksPerSecToExitSpeedMps(double ticksPerSec) {
        double revPerSec = ticksPerSec / SHOOTER_PARAMS.FLYWHEEL_TICKS_PER_REV;
        double angularVel = revPerSec * 2 * Math.PI;
        return angularVel * SHOOTER_PARAMS.FLYWHEEL_RADIUS_METERS * SHOOTER_PARAMS.GRIP_COEFFICIENT;
    }

    public double mpsToTicksPerSec(double mps) {
        double wheelCircumference = 2 * Math.PI * SHOOTER_PARAMS.FLYWHEEL_RADIUS_METERS;
        double revPerSec = mps / wheelCircumference;
        double idealTicksPerSec = revPerSec * SHOOTER_PARAMS.FLYWHEEL_TICKS_PER_REV;
        return idealTicksPerSec  / SHOOTER_PARAMS.GRIP_COEFFICIENT;
    }

    public void setHoodPosition(double position) {
        hoodLeftServo.setPosition(position);
        hoodRightServo.setPosition(position);
    }

    @Override
    public void reset() {
    }

    @Override
    public void update(){
        switch (shooterState) {
            case OFF:
                setShooterPower(0);
                adjustment = 0;
                break;

            case UPDATE:
                updateShooterSystem(getExitPositionInches(robot.drive.localizer.getPose(), robot.turret.getTurretEncoder(), ballExitAngleRad), robot.turret.targetPose);
                break;

            case FIXED_VELOCITY_IN_AUTO:
                setShooterVelocityPID(1115);
                setHoodPosition(0.34);
                break;
        }
    }
    public Command shooterTrackerCommand(GamepadTracker g) {
        return new Command() {
            private int num = 0;
            private double lastTime = 0;
            @Override
            public void execute() {
                if (recordTimer.milliseconds() - lastTime > ShooterSpeedRecorder.recordIntervalMs
                        && num < ShooterSpeedRecorder.recordAmountForEachShot
                        && ShooterSpeedRecorder.getCurrentShot() < ShooterSpeedRecorder.numShotsToRecord) {
                    lastTime = recordTimer.milliseconds();
                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][num][0] = recordTimer.seconds();
                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][num][1] = shooterPID.getTarget();
                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][num][2] = getAvgMotorVelocity();
                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][num][3] = shooterMotorHigh.getPower();
                    num++;
                }
            }
            @Override
            public void end(boolean interrupted) {
                ShooterSpeedRecorder.incrementCurrentShot();
            }
            @Override
            public boolean isFinished() {
                return !g.gamepad.dpad_down || num >= ShooterSpeedRecorder.recordAmountForEachShot;
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Collections.emptySet();
            }
        };
    }

    public double getVelHigh() {
        return Math.abs(shooterMotorHigh.getVelocity());
    }
    public double getVelLow() {
        return Math.abs(shooterMotorLow.getVelocity());
    }

    @Override
    public void printInfo() {
        telemetry.addLine("SHOOTER------");
        telemetry.addData("state", shooterState);
        telemetry.addData("high motor vel", getVelHigh());
        telemetry.addData("low motor vel", getVelLow());
        telemetry.addData("pid target vel", shooterPID.getTarget());
        telemetry.addData("power high", shooterMotorHigh.getPower());
        telemetry.addData("power low", shooterMotorLow.getPower());

        telemetry.addLine();
        telemetry.addLine("HOOD------");
        telemetry.addData("ball exit angle rad", ballExitAngleRad);
        telemetry.addData("left servo pos", hoodLeftServo.getPosition());
        telemetry.addData("right servo pos", hoodRightServo.getPosition());
    }

    public double getAvgMotorVelocity() {
        return (getVelHigh() + getVelLow()) * 0.5;
    }

    public static Vector2d getExitPositionInches(Pose2d robotPose, int turretEncoder, double ballExitAngleRad) {
        double hoodAngleRad = Math.PI * 0.5 - ballExitAngleRad;
        Pose2d turretPose = Turret.getTurretPose(robotPose, turretEncoder);
        double shooterCombinedRadiusInches = (SHOOTER_PARAMS.FLYWHEEL_RADIUS_METERS + SHOOTER_PARAMS.BALL_RADIUS_METERS) / 0.0254;
        double offsetFromTurretInches = SHOOTER_PARAMS.FLYWHEEL_OFFSET_FROM_TURRET_INCHES - Math.cos(hoodAngleRad) * shooterCombinedRadiusInches;

        return new Vector2d(
                turretPose.position.x + offsetFromTurretInches * Math.cos(turretPose.heading.toDouble()),
                turretPose.position.y + offsetFromTurretInches * Math.sin(turretPose.heading.toDouble())
        );
    }
    public static double getExitHeightMeters(double ballExitAngleRad) {
        double hoodAngleRad = Math.PI * 0.5 - ballExitAngleRad;
        return SHOOTER_PARAMS.FLYWHEEL_HEIGHT_METERS + (SHOOTER_PARAMS.FLYWHEEL_RADIUS_METERS + SHOOTER_PARAMS.BALL_RADIUS_METERS) * Math.sin(hoodAngleRad);
    }
    public static double getHoodServoPosition(double ballExitAngleRadians, Telemetry telemetry) {
        double hoodAngleFromXAxisRadians = Math.PI * 0.5 - ballExitAngleRadians;
        double hoodExitAngleDeg = Range.clip(Math.toDegrees(hoodAngleFromXAxisRadians), HoodParams.minAngleDeg, HoodParams.maxAngleDeg);
        double hoodPivotAngleDeg = hoodExitAngleDeg + HOOD_PARAMS.hoodPivotAngleOffsetFromHoodExitAngleDeg;
        double totalLinearDistanceMm = -0.00125315 * Math.pow(hoodPivotAngleDeg, 2) + 0.858968 * hoodPivotAngleDeg + 63.03978;
        double linearDistanceToExtendMm = totalLinearDistanceMm - HOOD_PARAMS.restingDistanceMm;
        if (telemetry != null) {
            telemetry.addLine("HOOD SERVO POS CALCULATION");
            telemetry.addData("ball exit angle rad", ballExitAngleRadians);
            telemetry.addData("hood Angle from x", hoodAngleFromXAxisRadians);
            telemetry.addData("total linear distance mm", totalLinearDistanceMm);
        }
        return linearDistanceToExtendMm / HOOD_PARAMS.servoRangeMm;
    }
}
