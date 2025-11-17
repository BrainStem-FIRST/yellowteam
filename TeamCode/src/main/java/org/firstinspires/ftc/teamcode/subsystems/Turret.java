package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.PoseStorage;
import org.firstinspires.ftc.teamcode.utils.Vec;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class Turret extends Component {
    public static boolean useRelativeVelocityCorrection = true;
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
        public double predictVelocityBallExitSpeedThreshold = 0.2;
        public double predictVelocityMultiplier = 5;
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
    public Vec robotVelocity, relativeBallExitVelocity, globalBallExitVelocity;
    public Turret(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot){
        super(hardwareMap, telemetry, robot);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController = new PIDController(TURRET_PARAMS.bigKP, TURRET_PARAMS.bigKI, TURRET_PARAMS.bigKD);
        turretState = TurretState.CENTER;
        robotVelocity = new Vec(0, 0);
        relativeBallExitVelocity = new Vec(0, 0);
        globalBallExitVelocity = new Vec(0, 0);
    }

    @Override
    public void printInfo() {

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
    public double getTurretRelativeAngleRad() {
        double turretTicksPerRadian = (TURRET_PARAMS.TICKS_PER_REV) / (2 * Math.PI);
        return turretMotor.getCurrentPosition() / turretTicksPerRadian;
    }
    public Pose2d getTurretPose(Pose2d robotPose) {
        double robotHeading = robotPose.heading.toDouble();
        double xOffset = -Math.cos(robotHeading) * offsetFromCenter;
        double yOffset = -Math.sin(robotHeading) * offsetFromCenter;
        return new Pose2d(robotPose.position.x + xOffset, robotPose.position.y + yOffset, robotHeading + getTurretRelativeAngleRad());
    }
    public void poseTargetToTurretTicks (Pose2d robotPose, Pose2d targetPose) {
        double turretMax = Math.toRadians(90);
        double turretMin = Math.toRadians(-90);
        double turretTicksPerRadian = (TURRET_PARAMS.TICKS_PER_REV) / (2 * Math.PI);

        Pose2d turretPose = getTurretPose(robotPose);
        Vec turretToGoal = new Vec(targetPose.position.x - turretPose.position.x,
                targetPose.position.y - turretPose.position.y).normalize();
        double ballExitSpeed = robot.shooter.ticksPerSecToMps(-robot.shooter.shooterMotorHigh.getVelocity());

        double targetAngle;
        // if shooter speed is too slow, don't account for relative velocity
        if (ballExitSpeed > TURRET_PARAMS.predictVelocityBallExitSpeedThreshold && useRelativeVelocityCorrection) {
            globalBallExitVelocity = turretToGoal.mult(ballExitSpeed);
            robotVelocity = robot.drive.pinpoint().getMostRecentVelocity().vec().mult(0.0254 * TURRET_PARAMS.predictVelocityMultiplier);
            relativeBallExitVelocity = globalBallExitVelocity.sub(robotVelocity);
            targetAngle = Math.atan2(relativeBallExitVelocity.y, relativeBallExitVelocity.x);
        }
        else
            targetAngle = Math.atan2(turretToGoal.y, turretToGoal.x);

        double turretTargetAngle = targetAngle - robotPose.heading.toDouble();
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

    private void fineAdjustTurretWithTag(AprilTagDetection tag) {
        if (tag == null) return;

        double tagX = tag.ftcPose.x;
        double tagZ = tag.ftcPose.z;

        double angularOffset = Math.atan2(-tagX, tagZ);
        int currentTicks = getTurretEncoder();

        double turretTicksPerRadian = (TURRET_PARAMS.TICKS_PER_REV) / (2 * Math.PI);
        int adjustment = (int)(angularOffset * turretTicksPerRadian);

        int targetTicks = currentTicks + adjustment;
        targetTicks = Math.max(TURRET_PARAMS.RIGHT_BOUND, Math.min(targetTicks, TURRET_PARAMS.LEFT_BOUND));

        pidController.setTarget(0);
        double power = pidController.update(tagX);
        turretMotor.setPower(-power);
//        setTurretPosition(targetTicks);
    }

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
                poseTargetToTurretTicks(robot.drive.pinpoint().getNextPoseSimple(TURRET_PARAMS.lookAheadTime), targetPose);
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
        telemetry.addData("exit speed", robot.shooter.ticksPerSecToMps(-robot.shooter.shooterMotorHigh.getVelocity()));
    }
}
