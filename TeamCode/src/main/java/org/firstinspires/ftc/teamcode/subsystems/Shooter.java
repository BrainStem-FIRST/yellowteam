package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
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

    public static boolean ENABLE_TESTING = false;
    public static boolean useVelocity = false;
    public static double testingShootPower = 0.99, testingShootVelocity = 1300, testingHoodPosition = 0.5;
    public DcMotorEx shooterMotorLow; // encoders for this one are cooked
    public DcMotorEx shooterMotorHigh; // encoders only work for this one
    public ServoImplEx hoodLeftServo;
    public ServoImplEx hoodRightServo;
    public ShooterState shooterState;

    public static class ShooterParams {
        public double SHOOTER_HEIGHT = 12.89; // inches from floor to where ball ejects
        public double TARGET_HEIGHT = 48.00; // inches from floor to goal height into target
        // (the height of the front wall of the goal is 38.75 in)

        public double WALL_OFFSET = 30;
        public double FLYWHEEL_RADIUS = 0.050; // meters of radius of the flywheel
        public double BALL_RADIUS = 0.064;
        public double SLIP_COEFFICIENT = 0.4386;
        public double FLYWHEEL_TICKS_PER_REV = 32; // ticks in 1 rotation of the motor
        public double HOOD_INCREMENT = 0.1;
        public double CLOSE_SHOOTER_POWER = 0.7;
        public double FAR_SHOOTER_POWER = 0.9;
        public double ZONE_THRESHOLD = 100;
        public double B_CLOSE_VALUE = 800.29968;
        public double B_FAR_VALUE = 725;
        public double SLOPE_CLOSE_VALUE = 4.53882;
        public double SLOPE_FAR_VALUE = 4.03631;
        public double TARGET_VELOCITY = 1525; //1525
        public double VELOCITY_OFFSET = 1;
        public double VELOCITY_CORRECTION = 1;

        public double kP = 0.005;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.00056;
    }

    public static ShooterParams SHOOTER_PARAMS = new ShooterParams();
    private final PIDController shooterPID;
    private double lastVelocity = 0;
    private boolean wasAboveThreshold = true;
    private double servoPos = 0;
    public double adjustment = 0;
    private final ElapsedTime recordTimer;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        shooterMotorLow = hardwareMap.get(DcMotorEx.class, "lowShoot");
        shooterMotorLow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorLow.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotorHigh = hardwareMap.get(DcMotorEx.class, "highShoot");
        shooterMotorHigh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorHigh.setDirection(DcMotorSimple.Direction.FORWARD);

        hoodLeftServo = hardwareMap.get(ServoImplEx.class, "hoodLeft");
        hoodLeftServo.setPwmRange(new PwmControl.PwmRange(1000, 1800));

        hoodRightServo = hardwareMap.get(ServoImplEx.class, "hoodRight");
        hoodRightServo.setPwmRange(new PwmControl.PwmRange(1000, 1800));

        shooterPID = new PIDController(SHOOTER_PARAMS.kP, SHOOTER_PARAMS.kI, SHOOTER_PARAMS.kD);

        shooterState = ShooterState.OFF;
        recordTimer = new ElapsedTime();
    }

    public void setShooterPower(double power) {
        shooterMotorHigh.setPower(power);
        shooterMotorLow.setPower(power);
    }

    public void setShooterVelocityPID(double targetVelocityTicksPerSec) {
        shooterPID.setTarget(targetVelocityTicksPerSec + adjustment);

        double currentVelocity = -shooterMotorHigh.getVelocity();
        double pidOutput = -shooterPID.update(currentVelocity);

        double feedForward = SHOOTER_PARAMS.kF * targetVelocityTicksPerSec;
        double totalPower = pidOutput + feedForward;

        totalPower = Math.abs(Range.clip(totalPower, -1.0, 1.0));

        setShooterPower(totalPower);

        telemetry.addData("Shooter Target Vel", targetVelocityTicksPerSec);
        telemetry.addData("Shooter Current Vel", currentVelocity);
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
//        targetPose = turret.isRedAlliance ? new Pose2d(-62, 58, Math.toRadians(0)) :
//                new Pose2d(-62, -58, Math.toRadians(0));
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double distance = Math.hypot(deltaX, deltaY) + SHOOTER_PARAMS.WALL_OFFSET;

        telemetry.addData("Dist to Shooter", distance);

        double actualVelocityMps = ticksPerSecToMps(-shooterMotorHigh.getVelocity());
        telemetry.addData("MPS", actualVelocityMps);

        double heightToTarget = (SHOOTER_PARAMS.TARGET_HEIGHT - SHOOTER_PARAMS.SHOOTER_HEIGHT);

        // Physics formula rearranged for angle: tan(θ) = (v² ± √(v⁴ - g(gx² + 2yh²))) / (gx)
        double g = 9.81;
        double x = distance * 0.0254; // convert inches to meters
        double y = heightToTarget * 0.0254; // convert inches to meters

        double robotVelToGoal = computeVelocityTowardGoal(robotPose, targetPose);
        double v = actualVelocityMps + robotVelToGoal * SHOOTER_PARAMS.VELOCITY_CORRECTION;
        double discriminant = v*v*v*v - g*(g*x*x + 2*y*v*v);
        if (discriminant <= 0)
            return Math.toRadians(40);

        double theta = Math.atan( (v*v - Math.sqrt(discriminant)) / (g * x) );
        telemetry.addData("OLD HORZ Angle", Math.toDegrees(theta));

        return theta;
    }

    public void setHoodFromAngle(double angleRadians) {
        double angleDeg = Math.toDegrees(angleRadians);
        angleDeg = Math.max(16.5, Math.min(45.1, angleDeg));
        servoPos = (-0.03497 * angleDeg) + 1.578; //angle(16.5-45.1) conversion to servo pos(1-0)

        setHoodPosition(servoPos);
    }

    public double computeVelocityTowardGoal(Pose2d robotPose, Pose2d targetPose) {
        double dx = targetPose.position.x - robotPose.position.x;
        double dy = targetPose.position.y - robotPose.position.y;

        double dist = Math.hypot(dx, dy);
        double ux = dx / dist;
        double uy = dy / dist;

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
        double distance = Math.hypot(deltaX, deltaY) + SHOOTER_PARAMS.WALL_OFFSET;

        double velocity = 0;
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
            velocity = testingShootVelocity;
        else {
            if (turretPose.position.x < 50)
                velocity = (SHOOTER_PARAMS.SLOPE_CLOSE_VALUE * distance) + SHOOTER_PARAMS.B_CLOSE_VALUE;
            else
                velocity = SHOOTER_PARAMS.TARGET_VELOCITY;
        }

        if (!ENABLE_TESTING || useVelocity)
            setShooterVelocityPID(velocity);
        else
            setShooterPower(testingShootPower);
        telemetry.addData("ORIGINAL POWER", original_power);

        if (ENABLE_TESTING)
            setHoodPosition(testingHoodPosition);
        else {
            double hoodAngle = calculateHoodAngle(turretPose, targetPose);
            if (turretPose.position.x < 50)
                setHoodFromAngle(hoodAngle);
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

        telemetry.addData("Dist to Shooter", distance);

        double actualVelocityMps = ticksPerSecToMps((shooterMotorLow.getVelocity() + shooterMotorHigh.getVelocity()) / 2.0);
        telemetry.addData("MPS", actualVelocityMps / SHOOTER_PARAMS.VELOCITY_OFFSET);
        double heightToTarget = (SHOOTER_PARAMS.TARGET_HEIGHT - SHOOTER_PARAMS.SHOOTER_HEIGHT);

        double g = 9.81;
        double r = (distance * 0.0254) * 2; // convert inches to meters
        double y = heightToTarget * 0.0254; // convert inches to meters

        double v = actualVelocityMps / SHOOTER_PARAMS.VELOCITY_OFFSET;

        double part1 = ((r*g)/(v*v)) * ((r*g)/(v*v));
        double part2 = Math.sqrt(1 - part1);
        double thetaHigh = 90 - Math.sinh((Math.sqrt((1 + part2)/2)));
        double thetaLow = 90 - Math.sinh((Math.sqrt((1 - part2)/2)));

        telemetry.addData("THETA HIGH", thetaHigh);
        telemetry.addData("THETA LOW", thetaLow);

        return 0;
    }

    public void farShotHoodUpdates(Pose2d robotPose, Pose2d targetPose) {
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double distance = Math.hypot(deltaX, deltaY) + SHOOTER_PARAMS.WALL_OFFSET;

        double power = (SHOOTER_PARAMS.SLOPE_FAR_VALUE * distance) + SHOOTER_PARAMS.B_FAR_VALUE;
        setShooterVelocityPID(power);

        double currentVelocity = shooterMotorHigh.getVelocity();

        if (wasAboveThreshold && (lastVelocity - currentVelocity) >= 250) {
            if (servoPos + SHOOTER_PARAMS.HOOD_INCREMENT <= 1.0) {
                servoPos += SHOOTER_PARAMS.HOOD_INCREMENT;
            }
            wasAboveThreshold = false;
        }

        if (currentVelocity > 1400) {
            wasAboveThreshold = true;
        }

        setHoodPosition(servoPos);

        lastVelocity = currentVelocity;
    }

    private void incrementHood(Pose2d robotPose, Pose2d targetPose) {
        targetPose = robot.alliance == Alliance.RED ? new Pose2d(-62, 58, Math.toRadians(0)) :
                new Pose2d(-62, -58, Math.toRadians(0));

        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double distance = Math.hypot(deltaX, deltaY) + SHOOTER_PARAMS.WALL_OFFSET;

        telemetry.addData("Dist to Shooter", distance);
        setShooterVelocityPID(SHOOTER_PARAMS.TARGET_VELOCITY);
        double hood = 0.025 * ((shooterMotorLow.getVelocity() + shooterMotorHigh.getVelocity()) / 2.0) + 36.16667;
        hood = 90 - hood;
        double servoPos1 = (-0.03497 * hood) + 1.578;
        setHoodPosition(servoPos1);
    }


    public double ticksPerSecToMps(double ticksPerSec) {
        double wheelCircumference = 2 * Math.PI * (SHOOTER_PARAMS.FLYWHEEL_RADIUS + SHOOTER_PARAMS.BALL_RADIUS);
        return (ticksPerSec / SHOOTER_PARAMS.FLYWHEEL_TICKS_PER_REV) * wheelCircumference * SHOOTER_PARAMS.SLIP_COEFFICIENT;
    }

    // doesn't account for slippage
    public double mpsToTicksPerSec(double mps) {
        double wheelCircumference = 2 * Math.PI * SHOOTER_PARAMS.FLYWHEEL_RADIUS;
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
                updateShooterSystem(robot.turret.getTurretPose(robot.drive.localizer.getPose()), robot.turret.targetPose);
                break;

            case SHOOT:
                setShooterVelocityPID(SHOOTER_PARAMS.TARGET_VELOCITY);
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

        telemetry.addData("SHOOTER POWER", shooterMotorHigh.getPower());
//        telemetry.addData("SHOOTER LOW ENCODER", shooterMotorLow.getCurrentPosition());
//        telemetry.addData("SHOOTER HIGH VELOCITY", shooterMotorHigh.getVelocity());
//        telemetry.addData("SHOOTER LOW VELOCITY", shooterMotorLow.getVelocity());
//        telemetry.addData("Shooter Adjustment Factor", adjustment);

        OdoInfo vel = robot.drive.pinpoint().getMostRecentVelocity();
        double vx = vel.x;
        double vy = vel.y;
        double angle = Math.toDegrees(vel.headingRad);
        telemetry.addData("X VELOCITY", vx);
        telemetry.addData("Y VELOCITY", vy);
        telemetry.addData("ANGLE VELOCITY DEG", angle);
        telemetry.addData("TOTAL VELOCITY", computeVelocityTowardGoal(robot.drive.localizer.getPose(), robot.turret.targetPose));
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


    @Override
    public void printInfo() {
    }
}
