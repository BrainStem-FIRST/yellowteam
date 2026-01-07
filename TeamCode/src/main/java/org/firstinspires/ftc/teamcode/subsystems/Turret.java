package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;
import org.firstinspires.ftc.teamcode.utils.math.Vec;

@Config
public class Turret extends Component {
    public static class TestingParams {
        public boolean actuallyPowerTurret = true;
    }
    public static class GoalParams {
        public double nearRedShotsGoalX = -64, nearRedShotsGoalY = 62, farRedShotsGoalX = -66, farRedShotsGoalY = 65;
        public double nearBlueShotsGoalX = -67, nearBlueShotsGoalY = -64, farBlueShotsGoalX = -67, farBlueShotsGoalY = -64;
    }
    public static class Params {
        public double offsetFromCenter = 3.742; // vertical offset of center of turret from center of robot in inches

        public int fineAdjust = 5;
        public double lookAheadTime = 0; // time to look ahead for pose prediction
        // variable deciding how to smooth out discontinuities in look ahead time
        public double startLookAheadSmoothValue = 1;
        public double endLookAheadSmoothValue = 0.;
        public double TICKS_PER_REV = 1228.5;
        public int RIGHT_BOUND = -320;
        public int LEFT_BOUND = 300;
    }
    public static class PowerTuning {
        public double rightKp = 0.0033, rightKi = 0, rightKd = 0.000; // old kD: 0.0005
        public double leftKp = 0.0033, leftKi = 0, leftKd = 0.000; // old kD: 0.001
        public double noPowerThreshold = 3;
        public double linearDistToResetKiThreshold = 1;
        public double headingDegToResetKiThreshold = 1;

        public int kDSignMult = -1;

        public double moveRightKf = -0.08;
        public double moveLeftKf = 0.08;
        public double inThresholdKfPower = 0.01;
    }
    public static TestingParams testingParams = new TestingParams();
    public static GoalParams goalParams = new GoalParams();
    public static Params turretParams = new Params();
    public static PowerTuning powerTuning = new PowerTuning();
    public enum TurretState {
        TRACKING, CENTER, PARK
    }
    public DcMotorEx turretMotor;
    private final PIDController pidController;
    public TurretState turretState;
    private int nearEncoderAdjustment, farEncoderAdjustment;
    public Pose2d targetPose;
    public Vec relativeBallExitVelocityMps, globalBallExitVelocityMps;
    public double targetAngleRad, currentAngleRad, turretAngleRad;
    public int targetEncoder;
    public double currentLookAheadTime;
    private double motorError;
    private double kF;
    public Turret(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot){
        super(hardwareMap, telemetry, robot);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0);

        pidController = new PIDController(powerTuning.rightKp, powerTuning.rightKi, powerTuning.rightKd);
        turretState = TurretState.CENTER;
        relativeBallExitVelocityMps = new Vec(0, 0);
        globalBallExitVelocityMps = new Vec(0, 0);
    }

    public int getTurretEncoder() {
        return turretMotor.getCurrentPosition();
    }

    public void setTurretPosition(int ticks, int currentEncoder) {
        targetEncoder = Range.clip(ticks, turretParams.RIGHT_BOUND, turretParams.LEFT_BOUND);
        motorError = currentEncoder - targetEncoder;
        boolean movingRight = motorError > 0;
        // within threshold - give 0 power
        if (Math.abs(motorError) <= powerTuning.noPowerThreshold) {
            turretMotor.setPower(-Math.signum(motorError) * powerTuning.inThresholdKfPower);
            return;
        }

        // reset kI whenever the robot moves significantly
        Pose2d prevPose = robot.drive.pinpoint().lastPose;
        Pose2d curPose = robot.drive.pinpoint().getPose();
        double positionChange = Math.hypot(curPose.position.x - prevPose.position.x, curPose.position.y - prevPose.position.y);
        double headingDegChange = Math.toDegrees(curPose.heading.toDouble() - prevPose.heading.toDouble());
        if (positionChange > powerTuning.linearDistToResetKiThreshold || headingDegChange >= powerTuning.headingDegToResetKiThreshold)
            pidController.reset();

        updatePIDFValues(movingRight, motorError);

        double newPower = -pidController.updateWithError(motorError);
        newPower += kF;

        if (currentEncoder < turretParams.RIGHT_BOUND)
            newPower = Math.max(newPower, 0);
        else if (currentEncoder > turretParams.LEFT_BOUND)
            newPower = Math.min(newPower, 0);

        if (testingParams.actuallyPowerTurret)
            turretMotor.setPower(newPower);
    }
    private void updatePIDFValues(boolean movingRight, double encoderError) {
        double kP = movingRight ? powerTuning.rightKp : powerTuning.leftKp;
        double kI = movingRight ? powerTuning.rightKi : powerTuning.leftKi;
        double kD = movingRight ? powerTuning.rightKd : powerTuning.leftKd;
        pidController.setPIDValues(kP, kI, kD);
        pidController.setPermanentKdSign(movingRight ? powerTuning.kDSignMult : -powerTuning.kDSignMult);
        if (movingRight)
            kF = powerTuning.moveRightKf;
        else
            kF = powerTuning.moveLeftKf;
//        double x1 = powerTuning.moveRightKfEncoder1, y1 = powerTuning.moveRightKf1;
//        double x2 = powerTuning.moveRightKfEncoder2, y2 = powerTuning.moveRightKf2;
//        if (!movingRight) {
//            x1 = powerTuning.moveLeftKfEncoder1;
//            y1 = powerTuning.moveLeftKf1;
//            x2 = powerTuning.moveLeftKfEncoder2;
//            y2 = powerTuning.moveLeftKf2;
//        }
//        TwoPointLine equation = new TwoPointLine(x1, y1, x2, y2);
//        telemetry.addData("TURRET KF SLOPE", equation.slope);
//        telemetry.addData("TURRET KF Y INT", equation.yInt);
//        kF = equation.calculate(currentEncoder);
        if (movingRight)
            kF = Math.min(kF, 0);
        else
            kF = Math.max(kF, 0);
    }

    public void resetEncoders() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static double getTurretRelativeAngleRad(int turretPosition) {
        double turretTicksPerRadian = (turretParams.TICKS_PER_REV) / (2 * Math.PI);
        return turretPosition / turretTicksPerRadian;
    }
    public static Pose2d getTurretPose(Pose2d robotPose, int turretPosition) {
        double robotHeading = robotPose.heading.toDouble();
        double xOffset = -Math.cos(robotHeading) * turretParams.offsetFromCenter;
        double yOffset = -Math.sin(robotHeading) * turretParams.offsetFromCenter;
        return new Pose2d(robotPose.position.x + xOffset, robotPose.position.y + yOffset, robotHeading + getTurretRelativeAngleRad(turretPosition));
    }

    public void updateLookAheadTime(boolean useLookAhead) {
        double newLookAhead = useLookAhead ? turretParams.lookAheadTime : 0;
        double smoothValue = useLookAhead ? turretParams.startLookAheadSmoothValue : turretParams.endLookAheadSmoothValue;
        currentLookAheadTime = MathUtils.lerp(currentLookAheadTime, newLookAhead, smoothValue);
    }

    private Pose2d getDefaultTargetGoalPose() {
        if (BrainSTEMRobot.alliance == Alliance.RED) {
            if (robot.shooter.isNear)
                return new Pose2d(goalParams.nearRedShotsGoalX, goalParams.nearRedShotsGoalY, 0);
            return new Pose2d(goalParams.farRedShotsGoalX, goalParams.farRedShotsGoalY, 0);
        }
        if (robot.shooter.isNear)
            return new Pose2d(goalParams.nearBlueShotsGoalX, goalParams.nearBlueShotsGoalY, 0);
        return new Pose2d(goalParams.farBlueShotsGoalX, goalParams.farBlueShotsGoalY, 0);
    }

    @Override
    public void update() {
        targetPose = getDefaultTargetGoalPose();
        Pose2d currentRobotPose = robot.drive.pinpoint().getPose();
        Pose2d futureRobotPose = robot.drive.pinpoint().getNextPoseSimple(currentLookAheadTime);
        double turretTicksPerRadian = (turretParams.TICKS_PER_REV) / (2 * Math.PI);
        int turretEncoder = getTurretEncoder();

        switch (turretState) {
            case TRACKING:
                if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
                    turretMotor.setPower(0);
                    break;
                }

                double ballExitAngleRad = robot.shooter.getBallExitAngleRad();
                Vector2d currentExitPosition = ShootingMath.calculateExitPositionInches(currentRobotPose, turretEncoder, ballExitAngleRad);
                Vector2d futureExitPosition = ShootingMath.calculateExitPositionInches(futureRobotPose, turretEncoder, ballExitAngleRad);
                double ballExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(robot.shooter.getAvgMotorVelocity(), ShootingMath.shooterSystemParams.powerLossCoefficient);
                double turretTargetAngleRad = ShootingMath.calculateTurretTargetAngleRad(targetPose, futureRobotPose, currentExitPosition, futureExitPosition, ballExitSpeedMps);
                targetAngleRad = turretTargetAngleRad + currentRobotPose.heading.toDouble();

                int targetTurretPosition = (int) (turretTargetAngleRad * turretTicksPerRadian);
                targetTurretPosition += robot.shooter.isNear ? nearEncoderAdjustment : farEncoderAdjustment;

                setTurretPosition(targetTurretPosition, turretEncoder);
                break;

            case CENTER:
                if (robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE) {
                    turretMotor.setPower(0);
                    break;
                }

                setTurretPosition(0, turretEncoder);
                targetAngleRad = currentRobotPose.heading.toDouble();
                break;

            case PARK:
                setTurretPosition(-330, turretEncoder);
                break;
        }

        turretAngleRad = turretEncoder / turretTicksPerRadian;
        currentAngleRad = turretAngleRad + currentRobotPose.heading.toDouble();
    }

    @Override
    public void printInfo() {
        double turretTicksPerDegree = turretParams.TICKS_PER_REV / 360.;
        int turretEncoder = getTurretEncoder();
        double angleDegError = motorError / turretTicksPerDegree;

        telemetry.addLine("TURRET------");
        telemetry.addData("target pose", MathUtils.formatPose2(targetPose));
        telemetry.addData("actually power turret", testingParams.actuallyPowerTurret);
        telemetry.addData("state", turretState);
        telemetry.addData("turret power", turretMotor.getPower());
//        telemetry.addData("turret kF", kF);
//        telemetry.addData("turret PID integral", pidController.getIntegral());
//        telemetry.addData("turret PID derivative", pidController.getDerivative());
        telemetry.addData("current encoder", turretEncoder);
        telemetry.addData("target encoder", targetEncoder);
        telemetry.addData("encoder error", motorError);
        telemetry.addData("moving right", motorError > 0);
        telemetry.addData("angle degree error", angleDegError);
        telemetry.addData("turret current relative angle deg", Math.toDegrees(turretAngleRad));
        telemetry.addData("turret current absolute angle deg", Math.toDegrees(currentAngleRad));
        telemetry.addData("turret target absolute angle deg", Math.toDegrees(targetAngleRad));
        telemetry.addData("look ahead time", currentLookAheadTime);
        telemetry.addData("exit position", getTurretPose(robot.drive.localizer.getPose(), turretEncoder));
    }

    public void changeEncoderAdjustment(int amount) {
        if(robot.shooter.isNear)
            nearEncoderAdjustment += amount;
        else
            farEncoderAdjustment += amount;
    }
}
