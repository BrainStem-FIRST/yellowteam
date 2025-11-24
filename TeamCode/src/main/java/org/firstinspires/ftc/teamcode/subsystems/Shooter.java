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
import org.firstinspires.ftc.teamcode.opmode.Alliance;
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
        public double SLIP_COEFFICIENT = 1;
        public double SHOOTER_MOTOR_TICKS_PER_REV = 28;

        // pulley ratio is 30:22
        public double FLYWHEEL_TICKS_PER_REV = SHOOTER_MOTOR_TICKS_PER_REV * 30 / 22; // ticks in 1 rotation of flywheel
        public double HOOD_INCREMENT = 0.1;
        public double CLOSE_SHOOTER_POWER = 0.99;
        public double FAR_SHOOTER_POWER = 0.9;
        public double ZONE_THRESHOLD = 100;
        public double B_CLOSE_VALUE = 800.29968;
        public double B_FAR_VALUE = 725;
        public double SLOPE_CLOSE_VALUE = 4.53882;
        public double SLOPE_FAR_VALUE = 4.03631;
        public double TARGET_VELOCITY_TICKS = 1525; //1525
        public double VELOCITY_OFFSET = 1;
        public double VELOCITY_CORRECTION = 1;

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
    public static ShooterParams SHOOTER_PARAMS = new ShooterParams();
    public static HoodParams HOOD_PARAMS = new HoodParams();
    public static boolean ENABLE_TESTING = false;
    public static boolean useVelocity = false;
    public static double testingShootPower = -0.99, testingShootVelocity = 1300, testingBallExitAngleRad = 0.7;
    public DcMotorEx shooterMotorLow; // encoders for this one are cooked
    public DcMotorEx shooterMotorHigh; // encoders only work for this one
    public ServoImplEx hoodLeftServo;
    public ServoImplEx hoodRightServo;
    public ShooterState shooterState;

    private final PIDController shooterPID;
    private double lastVelocity = 0;
    private boolean wasAboveThreshold = true;
    private double ballExitAngleRad;
    private double hoodServoPos;
    private double distanceFromGoal;
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

        double currentVelocity = Math.abs(shooterMotorHigh.getVelocity());
        double pidOutput = -shooterPID.update(currentVelocity);
        telemetry.addData("pid output", pidOutput);
        double feedForward = SHOOTER_PARAMS.kF * targetVelocityTicksPerSec;
        double totalPower = pidOutput + feedForward;

        totalPower = Math.abs(Range.clip(totalPower, -0.99, 0.99));
        telemetry.addData("total power", totalPower);

        setShooterPower(totalPower);
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

    public double calculateBallExitAngleRad(Pose2d robotPose, Pose2d targetPose, double distance) {
        double actualVelocityMps = ticksPerSecToFlywheelMps(Math.abs(shooterMotorHigh.getVelocity()));

        double exitHeightMeters = getExitHeightMeters(ballExitAngleRad);
        double heightToTargetMeters = (SHOOTER_PARAMS.TARGET_HEIGHT_INCHES * 0.0254 - exitHeightMeters);

        // Physics formula rearranged for angle: tan(θ) = (v² ± √(v⁴ - g(gx² + 2yv²))) / (gx)
        double g = 9.81;
        double x = distance * 0.0254; // convert inches to meters
        double y = heightToTargetMeters; // convert inches to meters

        double robotVelToGoal = computeVelocityTowardGoal(robotPose, targetPose);
        double v = actualVelocityMps + robotVelToGoal * SHOOTER_PARAMS.VELOCITY_CORRECTION;
        double discriminant = v*v*v*v - g*(g*x*x + 2*y*v*v);
        if (discriminant <= 0)
            return Math.toRadians(40);

        double theta = Math.atan( (v*v - Math.sqrt(discriminant)) / (g * x) );

        return theta;
    }
    public void setHoodFromAngle(double angleRadians) {
    }
    // old crucible code
//    public void setHoodFromAngle(double angleRadians) {
//        double angleDeg = Math.toDegrees(angleRadians);
//        angleDeg = Math.max(16.5, Math.min(45.1, angleDeg));
//        hoodServoPos = (-0.03497 * angleDeg) + 1.578; //angle(16.5-45.1) conversion to servo pos(1-0)
//
//        setHoodPosition(hoodServoPos);
//    }

    public double computeVelocityTowardGoal(Pose2d robotPose, Pose2d targetPose) {
        double dx = targetPose.position.x - robotPose.position.x;
        double dy = targetPose.position.y - robotPose.position.y;

        distanceFromGoal = Math.hypot(dx, dy);
        double ux = dx / distanceFromGoal;
        double uy = dy / distanceFromGoal;

        OdoInfo vel = robot.drive.pinpoint().getMostRecentVelocity();
        double vx = vel.x;
        double vy = vel.y;

        return vx*ux + vy*uy;
    }

    public void updateShooterSystem(Pose2d turretPose, Pose2d targetPose) {

//        targetPose = turret.isRedAlliance ? new Pose2d(-62, 58, Math.toRadians(0)) :
//                new Pose2d(-62, -58, Math.toRadians(0));
        double deltaX = targetPose.position.x - turretPose.position.x;
        double deltaY = targetPose.position.y - turretPose.position.y;
        double distance = Math.hypot(deltaX, deltaY) + SHOOTER_PARAMS.WALL_OFFSET_INCHES;

        double velocityTicks = 0;
        double original_power = 0;

//        if (robotPose.position.x < 50) {
//            original_power = (SHOOTER_PARAMS.SLOPE_CLOSE_VALUE * distance) + SHOOTER_PARAMS.B_CLOSE_VALUE;
//            double shooter_mps = ticksPerSecToMps(original_power);
//            double robot_mps = computeVelocityTowardGoal(robotPose, targetPose) * 0.0254; // actually returns negative value
//            double needed_mps = shooter_mps + (robot_mps * SHOOTER_PARAMS.VELOCITY_CORRECTION);
//            telemetry.addData("ADJUSTED POWER", mpsToTicksPerSec(needed_mps));
//            telemetry.addData("ROBOT MPS", robot_mps);
//            if (needed_mps < 0) needed_mps = 0;
//            power = mpsToTicksPerSec(needed_mps);
//        }
        if (ENABLE_TESTING)
            velocityTicks = testingShootVelocity;
        else {
            if (turretPose.position.x < 50)
                velocityTicks = (SHOOTER_PARAMS.SLOPE_CLOSE_VALUE * distance) + SHOOTER_PARAMS.B_CLOSE_VALUE;
            else
                velocityTicks = SHOOTER_PARAMS.TARGET_VELOCITY_TICKS;
        }

        if (!ENABLE_TESTING || useVelocity)
            setShooterVelocityPID(velocityTicks);
        else
            setShooterPower(testingShootPower);

        if (ENABLE_TESTING) {
            hoodServoPos = getHoodServoPosition(testingBallExitAngleRad, telemetry);
            setHoodPosition(hoodServoPos);
        }
        else {
            ballExitAngleRad = calculateBallExitAngleRad(turretPose, targetPose, distance);
            if (turretPose.position.x < 50) {
                hoodServoPos = getHoodServoPosition(ballExitAngleRad, telemetry);
                setHoodPosition(hoodServoPos);
            }
            else
                setHoodPosition(1);
        }
    }

    public void updateShooterSystemPART2(Pose2d robotPose, Pose2d targetPose) {

        targetPose = robot.alliance == Alliance.RED ? new Pose2d(-62, 58, Math.toRadians(0)) :
                new Pose2d(-62, -58, Math.toRadians(0));
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double distance = Math.hypot(deltaX, deltaY);

        double power = 0;

        if (robotPose.position.x < 20)
            power = (SHOOTER_PARAMS.SLOPE_CLOSE_VALUE * distance) + SHOOTER_PARAMS.B_CLOSE_VALUE;
        else
            power = (SHOOTER_PARAMS.SLOPE_FAR_VALUE * distance) + SHOOTER_PARAMS.B_FAR_VALUE;

        setShooterVelocityPID(power);

        calculateHoodAnglePART2(robotPose, targetPose);
        double hoodAngle = calculateHoodAnglePART2(robotPose, targetPose);
//        setHoodFromAngle(hoodAngle);
    }

    public double calculateHoodAnglePART2(Pose2d robotPose, Pose2d targetPose) {

        targetPose = robot.alliance == Alliance.RED ? new Pose2d(-62, 58, Math.toRadians(0)) :
                new Pose2d(-62, -58, Math.toRadians(0));
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double distance = Math.hypot(deltaX, deltaY);

        double actualVelocityMps = ticksPerSecToFlywheelMps((shooterMotorLow.getVelocity() + shooterMotorHigh.getVelocity()) / 2.0);
        double heightToTargetMeters = SHOOTER_PARAMS.TARGET_HEIGHT_INCHES * 0.0254 - SHOOTER_PARAMS.FLYWHEEL_HEIGHT_METERS;

        double g = 9.81;
        double r = (distance * 0.0254) * 2; // convert inches to meters
        double y = heightToTargetMeters; // convert inches to meters

        double v = actualVelocityMps / SHOOTER_PARAMS.VELOCITY_OFFSET;

        double part1 = ((r*g)/(v*v)) * ((r*g)/(v*v));
        double part2 = Math.sqrt(1 - part1);
        double thetaHigh = 90 - Math.sinh((Math.sqrt((1 + part2)/2)));
        double thetaLow = 90 - Math.sinh((Math.sqrt((1 - part2)/2)));

        return 0;
    }

    public void farShotHoodUpdates(Pose2d robotPose, Pose2d targetPose) {
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double distance = Math.hypot(deltaX, deltaY) + SHOOTER_PARAMS.WALL_OFFSET_INCHES;

        double power = (SHOOTER_PARAMS.SLOPE_FAR_VALUE * distance) + SHOOTER_PARAMS.B_FAR_VALUE;
        setShooterVelocityPID(power);

        double currentVelocity = shooterMotorHigh.getVelocity();

        if (wasAboveThreshold && (lastVelocity - currentVelocity) >= 250) {
            if (hoodServoPos + SHOOTER_PARAMS.HOOD_INCREMENT <= 1.0) {
                hoodServoPos += SHOOTER_PARAMS.HOOD_INCREMENT;
            }
            wasAboveThreshold = false;
        }

        if (currentVelocity > 1400) {
            wasAboveThreshold = true;
        }

        setHoodPosition(hoodServoPos);

        lastVelocity = currentVelocity;
    }

    private void incrementHood(Pose2d robotPose, Pose2d targetPose) {
        targetPose = robot.alliance == Alliance.RED ? new Pose2d(-62, 58, Math.toRadians(0)) :
                new Pose2d(-62, -58, Math.toRadians(0));

        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double distance = Math.hypot(deltaX, deltaY) + SHOOTER_PARAMS.WALL_OFFSET_INCHES;

        setShooterVelocityPID(SHOOTER_PARAMS.TARGET_VELOCITY_TICKS);
        double hood = 0.025 * ((shooterMotorLow.getVelocity() + shooterMotorHigh.getVelocity()) / 2.0) + 36.16667;
        hood = 90 - hood;
        double servoPos1 = (-0.03497 * hood) + 1.578;
        setHoodPosition(servoPos1);
    }


    public static double ticksPerSecToFlywheelMps(double ticksPerSec) {
        double revPerSec = ticksPerSec / SHOOTER_PARAMS.FLYWHEEL_TICKS_PER_REV;
        double angularVel = revPerSec * 2 * Math.PI;
        return angularVel * SHOOTER_PARAMS.FLYWHEEL_RADIUS_METERS * SHOOTER_PARAMS.SLIP_COEFFICIENT;
    }

    // doesn't account for slippage
    public double mpsToTicksPerSec(double mps) {
        double wheelCircumference = 2 * Math.PI * SHOOTER_PARAMS.FLYWHEEL_RADIUS_METERS;
        return (mps / wheelCircumference) * SHOOTER_PARAMS.FLYWHEEL_TICKS_PER_REV;
    }

    public void setHoodPosition(double position) {
        hoodLeftServo.setPosition(position);
        hoodRightServo.setPosition(position);
    }

    public enum ShooterState {
        OFF, SHOOT, UPDATE, FAR_SHOT_HOOD_UPDATES, UPDATE_2, AUTO_VELOCITY
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
                updateShooterSystem(Turret.getTurretPose(robot.drive.localizer.getPose(), robot.turret.getTurretEncoder()), robot.turret.targetPose);
                break;

            case SHOOT:
                setShooterVelocityPID(SHOOTER_PARAMS.TARGET_VELOCITY_TICKS);
//                calculateHoodAngle(drive.localizer.getPose(), turret.targetPose);
//                incrementHood(drive.localizer.getPose(), turret.targetPose);
                break;

            case AUTO_VELOCITY:
                setShooterVelocityPID(1115);
                setHoodPosition(0.34);
                break;

            case FAR_SHOT_HOOD_UPDATES:
                farShotHoodUpdates(robot.drive.localizer.getPose(), robot.turret.targetPose);
                break;

            case UPDATE_2:
                updateShooterSystemPART2(robot.drive.localizer.getPose(), robot.turret.targetPose);
                break;
        }


        OdoInfo vel = robot.drive.pinpoint().getMostRecentVelocity();
        double vx = vel.x;
        double vy = vel.y;
        double angle = Math.toDegrees(vel.headingRad);
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
                    ShooterSpeedRecorder.data[ShooterSpeedRecorder.getCurrentShot()][num][2] = -shooterMotorHigh.getVelocity();
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
        return shooterMotorHigh.getVelocity();
    }
    public double getVelLow() {
        return shooterMotorLow.getVelocity();
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
}
