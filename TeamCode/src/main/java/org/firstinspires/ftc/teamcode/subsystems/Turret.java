package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;
import org.firstinspires.ftc.teamcode.utils.misc.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.math.Vec;

@Config
public class Turret extends Component {
    public static double offsetFromCenter = 3.742; // vertical offset of center of turret from center of robot in inches
    public static class Params {
        public double goalX = -64, goalY = 64;
        public double bigKP = 0.0065, bigKI = 0, bigKD = 0.0005;
        public double smallKP = 0.015, smallKI = 0, smallKD = 0.0003;
        public double smallPIDValuesErrorThreshold = 15; // if error is less than 20, switch to small pid values
        public double lookAheadTime = 0.1; // time to look ahead for pose prediction
        // variable deciding how to smooth out discontinuities in look ahead time
        public double startLookAheadSmoothValue = 1;
        public double endLookAheadSmoothValue = 0.2;
        public int TICKS_PER_REV = 1212;
        public int RIGHT_BOUND = -300;
        public int LEFT_BOUND = 300;
    }
    public static Params TURRET_PARAMS = new Turret.Params();
    public enum TurretState {
        OFF, TRACKING, CENTER, PARK, RESET
    }
    public DcMotorEx turretMotor;
    private final PIDController pidController;
    public TurretState turretState;
    public int adjustment = 0;
    public Pose2d targetPose;
    public Vec exitVelocityMps, relativeBallExitVelocityMps, globalBallExitVelocityMps;
    public double targetAngleRad, currentAngleRad;
    public int targetEncoder;
    public double currentLookAheadTime;
    public Turret(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot){
        super(hardwareMap, telemetry, robot);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController = new PIDController(TURRET_PARAMS.bigKP, TURRET_PARAMS.bigKI, TURRET_PARAMS.bigKD);
        turretState = TurretState.CENTER;
        exitVelocityMps = new Vec(0, 0);
        relativeBallExitVelocityMps = new Vec(0, 0);
        globalBallExitVelocityMps = new Vec(0, 0);
        targetPose = getDefaultTargetGoalPose();
    }

    @Override
    public void printInfo() {
        double turretTicksPerDegree = TURRET_PARAMS.TICKS_PER_REV / 360.;
        double encoderError = targetEncoder - getTurretEncoder();
        double angleDegError = encoderError / turretTicksPerDegree;

        telemetry.addLine("TURRET------");
        telemetry.addData("state", turretState);
        telemetry.addData("encoder error", encoderError);
        telemetry.addData("angle degree error", angleDegError);
        telemetry.addData("current angle deg", Math.toDegrees(currentAngleRad));
        telemetry.addData("target angle deg", Math.toDegrees(targetAngleRad));
        telemetry.addData("look ahead time", currentLookAheadTime);

//        if(globalBallExitVelocityMps != null && relativeBallExitVelocityMps != null) {
//            double globalA = Math.atan2(globalBallExitVelocityMps.y, globalBallExitVelocityMps.x);
//            double localA = Math.atan2(relativeBallExitVelocityMps.y, relativeBallExitVelocityMps.x);
//            telemetry.addData("global exit angle", globalA);
//            telemetry.addData("relative exit angle", localA);
//            telemetry.addData("offset", globalA - localA);
//        }
    }

    public int getTurretEncoder() {
        return turretMotor.getCurrentPosition();
    }

    public void setTurretPosition(int ticks) {
        targetEncoder = ticks;
        double error = getTurretEncoder() - ticks;

        // use correct pid values based on error
        if (Math.abs(error) < TURRET_PARAMS.smallPIDValuesErrorThreshold)
            pidController.setPIDValues(TURRET_PARAMS.smallKP, TURRET_PARAMS.smallKI, TURRET_PARAMS.smallKD);
        else
            pidController.setPIDValues(TURRET_PARAMS.bigKP, TURRET_PARAMS.bigKI, TURRET_PARAMS.bigKD);

        double newPower = -pidController.updateWithError(error);
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

    @Override
    public void reset() {}

    public void updateLookAheadTime(boolean useLookAhead) {
        double newLookAhead = useLookAhead ? TURRET_PARAMS.lookAheadTime : 0;
        double smoothValue = useLookAhead ? TURRET_PARAMS.startLookAheadSmoothValue : TURRET_PARAMS.endLookAheadSmoothValue;
        currentLookAheadTime = MathUtils.lerp(currentLookAheadTime, newLookAhead, smoothValue);
    }

    private Pose2d getDefaultTargetGoalPose() {
        return new Pose2d(TURRET_PARAMS.goalX, robot.alliance == Alliance.RED ? TURRET_PARAMS.goalY : -TURRET_PARAMS.goalY, 0);
    }

    @Override
    public void update(){
        targetPose = getDefaultTargetGoalPose();
        switch (turretState) {
            case RESET:
                setTurretPosition(-PoseStorage.currentTurretEncoder);
                if (Math.abs((getTurretEncoder() - PoseStorage.currentTurretEncoder)) < 5) {
                    turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turretState = TurretState.CENTER;
                }
                break;

            case OFF:
                break;
            case TRACKING:
                Pose2d currentRobotPose = robot.drive.pinpoint().getPose();
                Pose2d futureRobotPose = robot.drive.pinpoint().getNextPoseSimple(currentLookAheadTime);
                int turretEncoder = getTurretEncoder();
                double ballExitAngleRad = robot.shooter.getBallExitAngleRad();
                Vector2d currentExitPosition = ShootingMath.calculateExitPositionInches(currentRobotPose, turretEncoder, ballExitAngleRad);
                Vector2d futureExitPosition = ShootingMath.calculateExitPositionInches(futureRobotPose, turretEncoder, ballExitAngleRad);
                double ballExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(robot.shooter.getAvgMotorVelocity());
                double turretTargetAngleRad = ShootingMath.calculateTurretTargetAngleRad(targetPose, futureRobotPose, currentExitPosition, futureExitPosition, ballExitSpeedMps);
                targetAngleRad = turretTargetAngleRad + currentRobotPose.heading.toDouble();

                double turretTicksPerRadian = (TURRET_PARAMS.TICKS_PER_REV) / (2 * Math.PI);
                int targetTurretPosition = (int)(turretTargetAngleRad * turretTicksPerRadian);
                targetTurretPosition += adjustment;
                targetTurretPosition = Math.max(TURRET_PARAMS.RIGHT_BOUND, Math.min(targetTurretPosition, TURRET_PARAMS.LEFT_BOUND));

                setTurretPosition(targetTurretPosition);
                currentAngleRad = getTurretEncoder() / turretTicksPerRadian + currentRobotPose.heading.toDouble();
                break;

            case CENTER:
                setTurretPosition(-PoseStorage.currentTurretEncoder);
                adjustment = 0;
                break;

            case PARK:
                setTurretPosition(-330);
                break;
        }
    }
}
