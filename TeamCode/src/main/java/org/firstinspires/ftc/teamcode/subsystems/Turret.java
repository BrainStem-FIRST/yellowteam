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
    public static boolean useRelativeVelocityCorrection = false;
    public static double offsetFromCenter = 3.742; // vertical offset of center of turret from center of robot in inches
    public static class Params{
        public double bigKP = 0.0065, bigKI = 0, bigKD = 0.0005;
        public double smallKP = 0.013, smallKI = 0, smallKD = 0.0003;
        public double smallPIDValuesErrorThreshold = 15; // if error is less than 20, switch to small pid values
        public double lookAheadTime = 0.09; // time to look ahead for pose prediction
        public double smoothPowerLerpValue = 1, useSmoothLerpValuePowerDiffThreshold = 0.6;
        public int TICKS_PER_REV = 1212;
        public int RIGHT_BOUND = -300;
        public int LEFT_BOUND = 300;
        public double predictVelocityExitSpeedThresholdInchesPerSec = 20; // the ball exit position must be traveling at a speed (in/sec) greater than this to account for its relative velocity
        public double predictVelocityMultiplier = 1;
    }
    public static Params TURRET_PARAMS = new Turret.Params();
    public enum TurretState {
        OFF, TRACKING, CENTER, PARK, RESET
    }
    public DcMotorEx turretMotor;
    private final PIDController pidController;
    public TurretState turretState;
    public int adjustment = 0;
    public Pose2d targetPose = new Pose2d(-62, 62, 0);
    public Vec exitVelocityMps, relativeBallExitVelocityMps, globalBallExitVelocityMps, ballExitLinearVelocityInchesPerSec;
    public double ballExitSpeedMps;
    public double targetAngleRad;
    public Turret(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot){
        super(hardwareMap, telemetry, robot);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController = new PIDController(TURRET_PARAMS.bigKP, TURRET_PARAMS.bigKI, TURRET_PARAMS.bigKD);
        turretState = TurretState.CENTER;
        exitVelocityMps = new Vec(0, 0);
        relativeBallExitVelocityMps = new Vec(0, 0);
        globalBallExitVelocityMps = new Vec(0, 0);
    }

    @Override
    public void printInfo() {
        telemetry.addLine("TURRET------");
        telemetry.addData("state", turretState);
        telemetry.addData("ball exit speed", ballExitSpeedMps);
        if(ballExitLinearVelocityInchesPerSec != null)
            telemetry.addData("turret vel", ballExitLinearVelocityInchesPerSec.mag());
        if(globalBallExitVelocityMps != null && relativeBallExitVelocityMps != null) {
            double globalA = Math.atan2(globalBallExitVelocityMps.y, globalBallExitVelocityMps.x);
            double localA = Math.atan2(relativeBallExitVelocityMps.y, relativeBallExitVelocityMps.x);
            telemetry.addData("global exit angle", globalA);
            telemetry.addData("relative exit angle", localA);
            telemetry.addData("offset", globalA - localA);
        }
    }

    public int getTurretEncoder() {
        return turretMotor.getCurrentPosition();
    }

    public void setTurretPosition(int ticks) {
        double error = getTurretEncoder() - ticks;

        // use correct pid values based on error
        if (Math.abs(error) < TURRET_PARAMS.smallPIDValuesErrorThreshold)
            pidController.setPIDValues(TURRET_PARAMS.smallKP, TURRET_PARAMS.smallKI, TURRET_PARAMS.smallKD);
        else
            pidController.setPIDValues(TURRET_PARAMS.bigKP, TURRET_PARAMS.bigKI, TURRET_PARAMS.bigKD);

        double oldPower = turretMotor.getPower();
        double newPower = -pidController.updateWithError(error);
        double smoothPower = newPower;
        if (oldPower!= 0 && Math.abs(newPower - oldPower) > TURRET_PARAMS.useSmoothLerpValuePowerDiffThreshold)
            smoothPower = MathUtils.lerp(oldPower, newPower, TURRET_PARAMS.smoothPowerLerpValue);

        turretMotor.setPower(smoothPower);
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
    // calculates turret angle with turret position
    /*
    public void poseTargetToTurretTicks (Pose2d currentRobotPose, Pose2d targetPose) {
        double turretMax = Math.toRadians(90);
        double turretMin = Math.toRadians(-90);
        double turretTicksPerRadian = (TURRET_PARAMS.TICKS_PER_REV) / (2 * Math.PI);

        Pose2d futureRobotPose = robot.drive.pinpoint().getNextPoseSimple(TURRET_PARAMS.lookAheadTime);
        Pose2d currentTurretPose = getTurretPose(currentRobotPose, getTurretEncoder());
        Pose2d futureTurretPose = getTurretPose(futureRobotPose, getTurretEncoder());

//        telemetry.addData("currentRobotPose", currentRobotPose);
//        telemetry.addData("targetPose", targetPose);
//        telemetry.addData("futureRobotPose", futureRobotPose);
//        telemetry.addData("currentTurretPose", currentTurretPose);
//        telemetry.addData("futureTurretPose", futureTurretPose);
        ballExitLinearVelocityInchesPerSec = new Vec(futureTurretPose.position.x - currentTurretPose.position.x, futureTurretPose.position.y - currentTurretPose.position.y);

        // predict turret position to account for turret lag
        Vec turretToGoal = new Vec(targetPose.position.x - futureTurretPose.position.x,
                targetPose.position.y - futureTurretPose.position.y).normalize();

        double targetAngle;
        // only account for robot velocity if it is significant
        if (ballExitLinearVelocityInchesPerSec.mag() > TURRET_PARAMS.predictVelocityExitSpeedThresholdInchesPerSec && useRelativeVelocityCorrection) {
            // find speed of ball relative to the ground (magnitude only)
            ballExitSpeedMps = Shooter.ticksPerSecToFlywheelMps(-robot.shooter.shooterMotorHigh.getVelocity());

            // find velocity of ball relative to the ground (direction and magnitude)
            globalBallExitVelocityMps = turretToGoal.mult(ballExitSpeedMps);

            // find robot velocity in meters and apply empirical multiplier
            exitVelocityMps = ballExitLinearVelocityInchesPerSec.mult(0.0254 * TURRET_PARAMS.predictVelocityMultiplier);

            // velocity of ball relative to robot = velocity of ball relative to ground - velocity of robot relative to ground
            relativeBallExitVelocityMps = globalBallExitVelocityMps.sub(exitVelocityMps);

            // find angle to shoot at relative velocity
            targetAngle = Math.atan2(relativeBallExitVelocityMps.y, relativeBallExitVelocityMps.x);
        }
        else
            // find angle to shoot at normally
            targetAngle = Math.atan2(turretToGoal.y, turretToGoal.x);

        double turretTargetAngle = targetAngle - currentRobotPose.heading.toDouble();
        turretTargetAngle = Math.atan2(Math.sin(turretTargetAngle), Math.cos(turretTargetAngle));

        if (turretTargetAngle > turretMax)
            turretTargetAngle = Math.PI - turretTargetAngle; //mirror angle
        else if (turretTargetAngle < turretMin)
            turretTargetAngle = -Math.PI - turretTargetAngle;

        int targetTurretPosition = (int)(turretTargetAngle * turretTicksPerRadian);

        targetTurretPosition += adjustment;

        targetTurretPosition = Math.max(TURRET_PARAMS.RIGHT_BOUND, Math.min(targetTurretPosition, TURRET_PARAMS.LEFT_BOUND));
        setTurretPosition(targetTurretPosition);
    }

     */

    // calculates turret angle with ball exit position
    public void poseTargetToTurretTicks (Pose2d currentRobotPose, Pose2d targetPose) {
        double turretMax = Math.toRadians(90);
        double turretMin = Math.toRadians(-90);
        double turretTicksPerRadian = (TURRET_PARAMS.TICKS_PER_REV) / (2 * Math.PI);

        Pose2d futureRobotPose = robot.drive.pinpoint().getNextPoseSimple(TURRET_PARAMS.lookAheadTime);
        Vector2d currentExitPosition = Shooter.getExitPositionInches(currentRobotPose, getTurretEncoder(), robot.shooter.getBallExitAngleRad());
        Vector2d futureExitPosition = Shooter.getExitPositionInches(futureRobotPose, getTurretEncoder(), robot.shooter.getBallExitAngleRad());

//        telemetry.addData("currentRobotPose", currentRobotPose);
//        telemetry.addData("targetPose", targetPose);
//        telemetry.addData("futureRobotPose", futureRobotPose);
//        telemetry.addData("currentTurretPose", currentTurretPose);
//        telemetry.addData("futureTurretPose", futureTurretPose);
        ballExitLinearVelocityInchesPerSec = new Vec(futureExitPosition.x - currentExitPosition.x, futureExitPosition.y - currentExitPosition.y);

        // predict turret position to account for turret lag
        Vec ballExitToGoal = new Vec(targetPose.position.x - futureExitPosition.x,
                targetPose.position.y - futureExitPosition.y).normalize();

        // only account for robot velocity if it is significant
        if (ballExitLinearVelocityInchesPerSec.mag() > TURRET_PARAMS.predictVelocityExitSpeedThresholdInchesPerSec && useRelativeVelocityCorrection) {
            // find speed of ball relative to the ground (magnitude only)
            ballExitSpeedMps = Shooter.ticksPerSecToExitSpeedMps(robot.shooter.getAvgMotorVelocity());

            // find velocity of ball relative to the ground (direction and magnitude)
            globalBallExitVelocityMps = ballExitToGoal.mult(ballExitSpeedMps);

            // find exit velocity in meters and apply empirical multiplier
            exitVelocityMps = ballExitLinearVelocityInchesPerSec.mult(0.0254 * TURRET_PARAMS.predictVelocityMultiplier);

            // velocity of ball relative to robot = velocity of ball relative to ground - velocity of robot relative to ground
            relativeBallExitVelocityMps = globalBallExitVelocityMps.sub(exitVelocityMps);

            // find angle to shoot at relative velocity
            targetAngleRad = Math.atan2(relativeBallExitVelocityMps.y, relativeBallExitVelocityMps.x);
        }
        else
            // find angle to shoot at normally
            targetAngleRad = Math.atan2(ballExitToGoal.y, ballExitToGoal.x);

        double turretTargetAngleRad = targetAngleRad - currentRobotPose.heading.toDouble();
        turretTargetAngleRad = Math.atan2(Math.sin(turretTargetAngleRad), Math.cos(turretTargetAngleRad)); // wrap between -pi, pi

        if (turretTargetAngleRad > turretMax)
            turretTargetAngleRad = Math.PI - turretTargetAngleRad; //mirror angle
        else if (turretTargetAngleRad < turretMin)
            turretTargetAngleRad = -Math.PI - turretTargetAngleRad;

        int targetTurretPosition = (int)(turretTargetAngleRad * turretTicksPerRadian);

        targetTurretPosition += adjustment;

        targetTurretPosition = Math.max(TURRET_PARAMS.RIGHT_BOUND, Math.min(targetTurretPosition, TURRET_PARAMS.LEFT_BOUND));
        setTurretPosition(targetTurretPosition);
    }

//    private void fineAdjustTurretWithTag(AprilTagDetection tag) {
//        if (tag == null) return;
//
//        double tagX = tag.ftcPose.x;
//        double tagZ = tag.ftcPose.z;
//
//        double angularOffset = Math.atan2(-tagX, tagZ);
//        int currentTicks = getTurretEncoder();
//
//        double turretTicksPerRadian = (TURRET_PARAMS.TICKS_PER_REV) / (2 * Math.PI);
//        int adjustment = (int)(angularOffset * turretTicksPerRadian);
//
//        int targetTicks = currentTicks + adjustment;
//        targetTicks = Math.max(TURRET_PARAMS.RIGHT_BOUND, Math.min(targetTicks, TURRET_PARAMS.LEFT_BOUND));
//
//        pidController.setTarget(0);
//        double power = pidController.update(tagX);
//        turretMotor.setPower(-power);
//        setTurretPosition(targetTicks);
//    }

    @Override
    public void reset() {
    }

    @Override
    public void update(){
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
//                turretMotor.setPower(0);
                break;

            case TRACKING:
                poseTargetToTurretTicks(robot.drive.pinpoint().getPose(), targetPose);
                break;

            case CENTER:
                setTurretPosition(-PoseStorage.currentTurretEncoder);
                adjustment = 0;
                break;

            case PARK:
                setTurretPosition(-330);
                break;
        }

        if (robot.alliance == Alliance.RED)
            targetPose = new Pose2d(-62, 62, 0);
        else
            targetPose = new Pose2d(-62, -62, 0);
    }

    public void printInfo(Telemetry telemetry) {
        telemetry.addLine("TURRET");
        telemetry.addData("state", turretState);
        telemetry.addData("motor power", turretMotor.getPower());
        telemetry.addData("motor position", turretMotor.getCurrentPosition());
        telemetry.addData("motor velocity", turretMotor.getVelocity());
        telemetry.addData("exit speed (m/s)", Shooter.ticksPerSecToExitSpeedMps(robot.shooter.getAvgMotorVelocity()));
    }
}
