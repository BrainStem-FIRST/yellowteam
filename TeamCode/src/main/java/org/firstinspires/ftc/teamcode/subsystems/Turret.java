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
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;
import org.firstinspires.ftc.teamcode.utils.math.Vec;

@Config
public class Turret extends Component {
    public static boolean powerTurret = false;
    public static double offsetFromCenter = 3.742; // vertical offset of center of turret from center of robot in inches
    public static class Params {
        public int fineAdjust = 5;
        public double nearRedShotsGoalX = -68, nearRedShotsGoalY = 67.5, farRedShotsGoalX = -70, farRedShotsGoalY = 68;
        public double nearBlueShotsGoalX = -68, nearBlueShotsGoalY = -64.5, farBlueShotsGoalX = -70, farBlueShotsGoalY = -63.5;
        public double bigKP = 0.0065, bigKI = 0, bigKD = 0.0005;
        public double smallKP = 0.017, smallKI = 0, smallKD = 0.0003, smallKf = 0.01;
        public double smallPIDValuesErrorThreshold = 15; // if error is less than 20, switch to small pid values
        public double noPowerThreshold = 2;
        public double lookAheadTime = 0.115; // time to look ahead for pose prediction
        // variable deciding how to smooth out discontinuities in look ahead time
        public double startLookAheadSmoothValue = 1;
        public double endLookAheadSmoothValue = 0.2;
        public double TICKS_PER_REV = 1228.5;
        public int RIGHT_BOUND = -300;
        public int LEFT_BOUND = 300;
    }
    public static Params TURRET_PARAMS = new Turret.Params();
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
    public Turret(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot){
        super(hardwareMap, telemetry, robot);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0);

        pidController = new PIDController(TURRET_PARAMS.bigKP, TURRET_PARAMS.bigKI, TURRET_PARAMS.bigKD);
        turretState = TurretState.CENTER;
        relativeBallExitVelocityMps = new Vec(0, 0);
        globalBallExitVelocityMps = new Vec(0, 0);
    }

    @Override
    public void printInfo() {
        double turretTicksPerDegree = TURRET_PARAMS.TICKS_PER_REV / 360.;
        int turretEncoder = getTurretEncoder();
        double encoderError = targetEncoder - turretEncoder;
        double angleDegError = encoderError / turretTicksPerDegree;

        telemetry.addLine("TURRET------");
//        telemetry.addData("state", turretState);
        telemetry.addData("turret power", turretMotor.getPower());
//        telemetry.addData("current encoder", turretEncoder);
//        telemetry.addData("target encoder", targetEncoder);
//        telemetry.addData("encoder error", encoderError);
//        telemetry.addData("angle degree error", angleDegError);
//        telemetry.addData("turret angle deg", Math.toDegrees(turretAngleRad));
//        telemetry.addData("current absolute angle deg", Math.toDegrees(currentAngleRad));
//        telemetry.addData("target absolute angle deg", Math.toDegrees(targetAngleRad));
//        telemetry.addData("look ahead time", currentLookAheadTime);
//        telemetry.addData("EXIT POSITION", getTurretPose(robot.drive.localizer.getPose(), turretEncoder));
    }

    public int getTurretEncoder() {
        return turretMotor.getCurrentPosition();
    }

    public void setTurretPosition(int ticks, int currentEncoder) {
        if (!powerTurret) {
            turretMotor.setPower(0);
            return;
        }
        targetEncoder = Range.clip(ticks, TURRET_PARAMS.RIGHT_BOUND, TURRET_PARAMS.LEFT_BOUND);
        double error = currentEncoder - targetEncoder;

        // within threshold - give 0 power
        if (Math.abs(error) < TURRET_PARAMS.noPowerThreshold) {
            turretMotor.setPower(0);
            return;
        }

        // use correct pid values based on error
        if (Math.abs(error) < TURRET_PARAMS.smallPIDValuesErrorThreshold)
            pidController.setPIDValues(TURRET_PARAMS.smallKP, TURRET_PARAMS.smallKI, TURRET_PARAMS.smallKD);
        else
            pidController.setPIDValues(TURRET_PARAMS.bigKP, TURRET_PARAMS.bigKI, TURRET_PARAMS.bigKD);

        double newPower = -pidController.updateWithError(error);
        newPower += Math.signum(newPower) * TURRET_PARAMS.smallKf;

        if (currentEncoder < TURRET_PARAMS.RIGHT_BOUND)
            newPower = Math.max(newPower, 0);
        else if (currentEncoder > TURRET_PARAMS.LEFT_BOUND)
            newPower = Math.min(newPower, 0);

        turretMotor.setPower(newPower);
    }

    public void resetEncoders() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public static double getTurretRelativeAngleRad(int turretPosition) {
        double turretTicksPerRadian = (TURRET_PARAMS.TICKS_PER_REV) / (2 * Math.PI);
        return turretPosition / turretTicksPerRadian;
    }
    public static Pose2d getTurretPose(Pose2d robotPose, int turretPosition) {
        double robotHeading = robotPose.heading.toDouble();
        double xOffset = -Math.cos(robotHeading) * offsetFromCenter;
        double yOffset = -Math.sin(robotHeading) * offsetFromCenter;
        return new Pose2d(robotPose.position.x + xOffset, robotPose.position.y + yOffset, robotHeading + getTurretRelativeAngleRad(turretPosition));
    }

    public void updateLookAheadTime(boolean useLookAhead) {
        double newLookAhead = useLookAhead ? TURRET_PARAMS.lookAheadTime : 0;
        double smoothValue = useLookAhead ? TURRET_PARAMS.startLookAheadSmoothValue : TURRET_PARAMS.endLookAheadSmoothValue;
        currentLookAheadTime = MathUtils.lerp(currentLookAheadTime, newLookAhead, smoothValue);
    }

    private Pose2d getDefaultTargetGoalPose() {
        if (robot.alliance == Alliance.RED) {
            if (robot.shooter.isNear)
                return new Pose2d(TURRET_PARAMS.nearRedShotsGoalX, TURRET_PARAMS.nearRedShotsGoalY, 0);
            return new Pose2d(TURRET_PARAMS.farRedShotsGoalX, TURRET_PARAMS.farRedShotsGoalY, 0);
        }
        if (robot.shooter.isNear)
            return new Pose2d(TURRET_PARAMS.nearBlueShotsGoalX, TURRET_PARAMS.nearBlueShotsGoalY, 0);
        return new Pose2d(TURRET_PARAMS.farBlueShotsGoalX, TURRET_PARAMS.farBlueShotsGoalY, 0);

    }

    @Override
    public void update() {
        turretMotor.setPower(0);
        if (false) {

            targetPose = getDefaultTargetGoalPose();
            Pose2d currentRobotPose = robot.drive.pinpoint().getPose();
            Pose2d futureRobotPose = robot.drive.pinpoint().getNextPoseSimple(currentLookAheadTime);
            double turretTicksPerRadian = (TURRET_PARAMS.TICKS_PER_REV) / (2 * Math.PI);
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
}
