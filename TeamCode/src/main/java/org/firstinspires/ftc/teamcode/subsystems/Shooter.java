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
    public static double tempTicksPerSec = 1300;
    public static class ShooterParams {
        public double WALL_OFFSET_INCHES = 30;
        public double FAR_TARGET_VEL_TICKS = 1525;
        public double kP = 0.005;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.00056;
    }
    public static class HoodParams {
        public double downPWM = 900, upPWM = 2065;
    }
    public static ShooterParams SHOOTER_PARAMS = new ShooterParams();
    public static HoodParams HOOD_PARAMS = new HoodParams();
    public static boolean ENABLE_TESTING = false;

    public enum ShooterState {
        OFF, UPDATE, FIXED_VELOCITY_IN_AUTO
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
//    public double calculateBallExitAngleRad(Vector2d ballExitPosition, Pose2d targetPose, double distance) {
//        double ballExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(getAvgMotorVelocity());
//
//        // get the EXACT exit height of the ball (depends on hood position)
//        double exitHeightMeters = ShootingMath.calculateExitHeightMeters(ballExitAngleRad);
//        double heightToTargetMeters = (SHOOTER_PARAMS.TARGET_HEIGHT_INCHES * 0.0254 - exitHeightMeters);
//
//        // Physics formula rearranged for angle: tan(θ) = (v² ± √(v⁴ - g(gx² + 2yv²))) / (gx)
//        double g = 9.81;
//        double x = distance * 0.0254; // convert inches to meters
//        double y = heightToTargetMeters; // convert inches to meters
//
//        double v = ballExitSpeedMps;
//         if (useRelativeVelocity) {
//             double exitPositionSpeedTowardsGoalMps = ShootingMath.calculateSpeedTowardGoalMps(targetPose, ballExitPosition, robot.drive.pinpoint().getMostRecentVelocity());
//             v -= exitPositionSpeedTowardsGoalMps * SHOOTER_PARAMS.RELATIVE_VELOCITY_CORRECTION;
//         }
//        double discriminant = v*v*v*v - g*(g*x*x + 2*y*v*v);
//        if (discriminant <= 0)
//            return Math.toRadians(40);
//
//        double tanTheta = (v*v - Math.sqrt(discriminant)) / (g * x);
//        return Math.atan(tanTheta);
//    }
//    public double computeSpeedTowardGoalMps(Vector2d ballExitPosition, Pose2d targetPose) {
//        double dx = targetPose.position.x - ballExitPosition.x;
//        double dy = targetPose.position.y - ballExitPosition.y;
//
//        double distanceFromGoal = Math.hypot(dx, dy);
//        double ux = dx / distanceFromGoal;
//        double uy = dy / distanceFromGoal;
//
//        OdoInfo vel = robot.drive.pinpoint().getMostRecentVelocity();
//        double vx = vel.x;
//        double vy = vel.y;
//
//        return -(vx*ux + vy*uy) * 0.0254;
//    }

    // old update shooter system function
    /*
    public void updateShooterSystem(Vector2d ballExitPosition, Pose2d targetPose) {
        double deltaX = targetPose.position.x - ballExitPosition.x;
        double deltaY = targetPose.position.y - ballExitPosition.y;
        double inchesFromGoal = Math.hypot(deltaX, deltaY) + SHOOTER_PARAMS.WALL_OFFSET_INCHES;

        // update FLYWHEEL
        double velocityTicks;
        if (ballExitPosition.x < 50) {
            double absoluteExitSpeedTicksPerSec = (SHOOTER_PARAMS.SLOPE_CLOSE_VALUE * inchesFromGoal) + SHOOTER_PARAMS.B_CLOSE_VALUE;
            if (useRelativeVelocity) {
                double absoluteExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(absoluteExitSpeedTicksPerSec);
                // SHOULD return positive value
                double exitPositionSpeedTowardsGoalMps = ShootingMath.calculateSpeedTowardGoalMps(targetPose, ballExitPosition, robot.drive.pinpoint().getMostRecentVelocity());
                double relativeExitSpeedMps = absoluteExitSpeedMps - (exitPositionSpeedTowardsGoalMps * SHOOTER_PARAMS.RELATIVE_VELOCITY_CORRECTION);
                telemetry.addData("ADJUSTED POWER", ShootingMath.mpsToTicksPerSec(relativeExitSpeedMps));
                telemetry.addData("BALL EXIT MPS", exitPositionSpeedTowardsGoalMps);

                velocityTicks = ShootingMath.mpsToTicksPerSec(relativeExitSpeedMps);
            }
            else
                velocityTicks = absoluteExitSpeedTicksPerSec;
        }
        else
            velocityTicks = SHOOTER_PARAMS.FAR_TARGET_VEL_TICKS;
        setShooterVelocityPID(velocityTicks);


        // update HOOD
        if (ENABLE_TESTING) {
            hoodServoPos = ShootingMath.calculateHoodServoPosition(testingBallExitAngleRad, telemetry);
            setHoodPosition(hoodServoPos);
        }
        else {
            if (ballExitPosition.x < 50) {
                double ballExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(getAvgMotorVelocity());
                OdoInfo mostRecentVelocity = robot.drive.pinpoint().getMostRecentVelocity();
                ballExitAngleRad = ShootingMath.calculateBallExitAngleRad(targetPose, ballExitPosition, inchesFromGoal, ballExitSpeedMps, ballExitAngleRad, mostRecentVelocity);

                hoodServoPos = ShootingMath.calculateHoodServoPosition(ballExitAngleRad, telemetry);
                setHoodPosition(hoodServoPos);
            }
            else
                setHoodPosition(1);
        }
    }
     */
    public void updateShooterSystem(Vector2d ballExitPosition, Pose2d targetPose) {
        double deltaX = targetPose.position.x - ballExitPosition.x;
        double deltaY = targetPose.position.y - ballExitPosition.y;
        double inchesFromGoal = Math.hypot(deltaX, deltaY) + SHOOTER_PARAMS.WALL_OFFSET_INCHES;

        // update FLYWHEEL
        OdoInfo mostRecentVelocity = robot.drive.pinpoint().getMostRecentVelocity();
        double flywheelTicksPerSec = ShootingMath.calculateFlywheelSpeedTicksPerSec(telemetry, targetPose, inchesFromGoal, ballExitPosition, mostRecentVelocity);

        setShooterVelocityPID(flywheelTicksPerSec);

        // update HOOD
        if (ENABLE_TESTING) {
            hoodServoPos = ShootingMath.calculateHoodServoPosition(testingBallExitAngleRad, telemetry);
            setHoodPosition(hoodServoPos);
        }
        else {
            if (ballExitPosition.x < 50) {
                double ballExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(getAvgMotorVelocity());
                telemetry.addData("avg motor vel ticks/s", getAvgMotorVelocity());
                telemetry.addData("exit speed mps", ballExitSpeedMps);
                ballExitAngleRad = ShootingMath.calculateBallExitAngleRad(targetPose, ballExitPosition, inchesFromGoal, ballExitSpeedMps, ballExitAngleRad, mostRecentVelocity, telemetry);
                hoodServoPos = ShootingMath.calculateHoodServoPosition(ballExitAngleRad, telemetry);
                setHoodPosition(hoodServoPos);
            }
            else
                setHoodPosition(1);
        }
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
                Vector2d ballExitPosition = ShootingMath.calculateExitPositionInches(robot.drive.localizer.getPose(), robot.turret.getTurretEncoder(), ballExitAngleRad);
                updateShooterSystem(ballExitPosition, robot.turret.targetPose);
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
}
