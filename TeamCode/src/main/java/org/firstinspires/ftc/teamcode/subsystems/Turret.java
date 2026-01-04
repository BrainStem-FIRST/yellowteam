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
    public static double offsetFromCenter = 3.742; // vertical offset of center of turret from center of robot in inches
    public static class Params {
        public int fineAdjust = 5;
        public double nearRedShotsGoalX = -68, nearRedShotsGoalY = 67.5, farRedShotsGoalX = -70, farRedShotsGoalY = 68;
        public double nearBlueShotsGoalX = -68, nearBlueShotsGoalY = -64.5, farBlueShotsGoalX = -70, farBlueShotsGoalY = -63.5;
        public double lookAheadTime = 0.115; // time to look ahead for pose prediction
        // variable deciding how to smooth out discontinuities in look ahead time
        public double startLookAheadSmoothValue = 1;
        public double endLookAheadSmoothValue = 0.2;
        public double TICKS_PER_REV = 1228.5;
        public int RIGHT_BOUND = -300;
        public int LEFT_BOUND = 300;
    }
    public static class PowerTuning {
        public double bigKP = 0.003, bigKI = 0, bigKD = 0;
        public double smallKP = 0, smallKI = 0, smallKD = 0;
        public double smallPIDValuesErrorThreshold = 0;
        public double noPowerThreshold = 2;

        public double moveRightKfSlope = 0.000294795, moveRightKfYInt = -0.113908;
        public double moveLeftKfSlope = 0, moveLeftKfYInt = 0;


//        public double noTensionLowerBound = Math.toRadians(-20);
//        public double noTensionUpperBound = Math.toRadians(20);
//        public double staticFrictionMinPow = 0.1;
//        public double kineticFrictionMinPow = 0.1;

    }
    public static Params turretParams = new Params();
    public static PowerTuning powerTuning = new PowerTuning();
    public enum TurretState {
        TRACKING, CENTER, PARK
    }
    public DcMotorEx turretMotor;
    private final PIDController pidController;
    public TurretState turretState;
    public int adjustment = 0;
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

        pidController = new PIDController(powerTuning.bigKP, powerTuning.bigKI, powerTuning.bigKD);
        turretState = TurretState.CENTER;
        relativeBallExitVelocityMps = new Vec(0, 0);
        globalBallExitVelocityMps = new Vec(0, 0);
    }

    @Override
    public void printInfo() {
        double turretTicksPerDegree = turretParams.TICKS_PER_REV / 360.;
        int turretEncoder = getTurretEncoder();
        double angleDegError = motorError / turretTicksPerDegree;

        telemetry.addLine("TURRET------");
        telemetry.addData("state", turretState);
        telemetry.addData("turret power", turretMotor.getPower());
        telemetry.addData("turret kF", kF);
        telemetry.addData("current encoder", turretEncoder);
        telemetry.addData("target encoder", targetEncoder);
        telemetry.addData("encoder error", motorError);
        telemetry.addData("moving right", motorError > 0);
        telemetry.addData("angle degree error", angleDegError);
        telemetry.addData("turret angle deg", Math.toDegrees(turretAngleRad));
        telemetry.addData("current absolute angle deg", Math.toDegrees(currentAngleRad));
        telemetry.addData("target absolute angle deg", Math.toDegrees(targetAngleRad));
        telemetry.addData("look ahead time", currentLookAheadTime);
        telemetry.addData("exit position", getTurretPose(robot.drive.localizer.getPose(), turretEncoder));
    }

    public int getTurretEncoder() {
        return turretMotor.getCurrentPosition();
    }

    public void setTurretPosition(int ticks, int currentEncoder) {
        targetEncoder = Range.clip(ticks, turretParams.RIGHT_BOUND, turretParams.LEFT_BOUND);
        motorError = currentEncoder - targetEncoder;

        // within threshold - give 0 power
        if (Math.abs(motorError) <= powerTuning.noPowerThreshold) {
            turretMotor.setPower(0);
            return;
        }

        // use correct pid values based on error
        if (Math.abs(motorError) < powerTuning.smallPIDValuesErrorThreshold)
            pidController.setPIDValues(powerTuning.smallKP, powerTuning.smallKI, powerTuning.smallKD);
        else
            pidController.setPIDValues(powerTuning.bigKP, powerTuning.bigKI, powerTuning.bigKD);

        double newPower = -pidController.updateWithError(motorError);
        boolean movingRight = motorError > 0;
        if (movingRight) {
            kF = turretMotor.getCurrentPosition() * powerTuning.moveRightKfSlope + powerTuning.moveRightKfYInt;
            newPower += Math.min(kF, 0);
        }
        else {
            newPower += turretMotor.getCurrentPosition() * powerTuning.moveLeftKfSlope + powerTuning.moveLeftKfYInt;
        }

        if (currentEncoder < turretParams.RIGHT_BOUND)
            newPower = Math.max(newPower, 0);
        else if (currentEncoder > turretParams.LEFT_BOUND)
            newPower = Math.min(newPower, 0);

        turretMotor.setPower(newPower);
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
        double xOffset = -Math.cos(robotHeading) * offsetFromCenter;
        double yOffset = -Math.sin(robotHeading) * offsetFromCenter;
        return new Pose2d(robotPose.position.x + xOffset, robotPose.position.y + yOffset, robotHeading + getTurretRelativeAngleRad(turretPosition));
    }

    public void updateLookAheadTime(boolean useLookAhead) {
        double newLookAhead = useLookAhead ? turretParams.lookAheadTime : 0;
        double smoothValue = useLookAhead ? turretParams.startLookAheadSmoothValue : turretParams.endLookAheadSmoothValue;
        currentLookAheadTime = MathUtils.lerp(currentLookAheadTime, newLookAhead, smoothValue);
    }

    private Pose2d getDefaultTargetGoalPose() {
        if (robot.alliance == Alliance.RED) {
            if (robot.shooter.isNear)
                return new Pose2d(turretParams.nearRedShotsGoalX, turretParams.nearRedShotsGoalY, 0);
            return new Pose2d(turretParams.farRedShotsGoalX, turretParams.farRedShotsGoalY, 0);
        }
        if (robot.shooter.isNear)
            return new Pose2d(turretParams.nearBlueShotsGoalX, turretParams.nearBlueShotsGoalY, 0);
        return new Pose2d(turretParams.farBlueShotsGoalX, turretParams.farBlueShotsGoalY, 0);

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
                double ballExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(robot.shooter.getAvgMotorVelocity());
                double turretTargetAngleRad = ShootingMath.calculateTurretTargetAngleRad(targetPose, futureRobotPose, currentExitPosition, futureExitPosition, ballExitSpeedMps);
                targetAngleRad = turretTargetAngleRad + currentRobotPose.heading.toDouble();

                int targetTurretPosition = (int) (turretTargetAngleRad * turretTicksPerRadian);
                targetTurretPosition += adjustment;

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
}
