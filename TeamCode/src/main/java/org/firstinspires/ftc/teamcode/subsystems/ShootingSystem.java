package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterLookup.lookupDistsI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.math.Vector3d;
import org.firstinspires.ftc.teamcode.utils.misc.MotorCacher;
import org.firstinspires.ftc.teamcode.utils.misc.ServoImplCacher;

@Config
public class ShootingSystem {
    public static class GoalParams {
        public double nearRedShotsGoalX = -65, nearRedShotsGoalY = 65, farRedShotsGoalX = -66, farRedShotsGoalY = 65;
        public double nearBlueShotsGoalX = -66, nearBlueShotsGoalY = -65, farBlueShotsGoalX = -67, farBlueShotsGoalY = -63;
        public double highArcTargetHeightIn = 38, lowArcNearTargetHeightIn = 39, lowArcFarTargetHeightIn = 42;
    }
    public static GoalParams goalParams = new GoalParams();
    public static class HoodParams {
        public double downPWM = 900, upPWM = 2065;
        public double minExitAngRad = Math.toRadians(35), maxExitAngRad = Math.toRadians(85);
        public double resolution = 0.005;
    }
    public static class MiscParams {
        public boolean usingLookup = false;
        public boolean enableShootingWhileMoving = true;
        public int lookAheadAvgNum = 5;
        public double rawLookAheadTime = 0.2; // time to look ahead for pose prediction
        public double shooterTau = 0.2;
    }
    public static class MathParams {
        public double highArcDistThreshold = 63;
        public double numApproximations = 3;
        // efficiency coef regression: y=-0.0766393x+0.446492
        public double efficiencyCoefM = -0.0766393, efficiencyCoefB = 0.446492;
    }
    public static MiscParams miscParams = new MiscParams();
    public static HoodParams hoodParams = new HoodParams();
    public static MathParams mathParams = new MathParams();
    private final HardwareMap hardwareMap;
    private final BrainSTEMRobot robot;
    private MotorCacher turretMotor, shooterLowMotor, shooterHighMotor;
    private ServoImplCacher hoodLeftServo, hoodRightServo;

    public boolean isNear;
    public Vector2d targetPos;
    public Pose2d futureRobotPose;

    public double absoluteTargetAngleRad;

    public double currentLookAhead;
    public double[] prevLookAheads;

    public Vector2d robotVelAtExitPosIps;
    public double robotSpeedAtExitPosIps;
    public Vector3d actualTargetExitVelMps;
    public double actualTargetExitSpeedMps;
    public double ballTargetExitSpeedMps;
    public double efficiencyCoef;

    public double filteredShooterSpeedTps, rawShooterSpeedTps;
    public double curExitSpeedMps;
    public double ballExitAngleRad, hoodExitAngleRad, physicsExitAngleRad;
    public Vector2d ballExitPos;
    public double ballExitPosInchesFromGoal;
    public Pose2d turretPose;

    private double curTimeMs;
    public double dt;
    public OdoInfo odoVel;
    public ShooterLookup lookupTable;

    public boolean useHighArc;
    public double relTargetHeightM;
    public ShootingSystem(HardwareMap hardwareMap, BrainSTEMRobot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;

        if(robot != null)
            ballExitPos = ShootingMath.calculateExitPositionInches(ShootingMath.getTurretPose(robot.drive.localizer.getPose(), 0), ballExitAngleRad);

        initTurret();
        initShooter();
        initHood();
        initMisc();
        curTimeMs = System.currentTimeMillis();
        lookupTable = new ShooterLookup();
        efficiencyCoef = 0.39;
    }
    private void initTurret() {
        DcMotorEx rawTurretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        rawTurretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rawTurretMotor.setPower(0);
        this.turretMotor = new MotorCacher(rawTurretMotor);
    }
    private void initShooter() {
        DcMotorEx rawShooterMotorLow = hardwareMap.get(DcMotorEx.class, "lowShoot");
        rawShooterMotorLow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rawShooterMotorLow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rawShooterMotorLow.setDirection(DcMotorSimple.Direction.FORWARD);
        rawShooterMotorLow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.shooterLowMotor = new MotorCacher(rawShooterMotorLow);

        DcMotorEx rawShooterMotorHigh = hardwareMap.get(DcMotorEx.class, "highShoot");
        rawShooterMotorHigh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rawShooterMotorHigh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rawShooterMotorHigh.setDirection(DcMotorSimple.Direction.REVERSE);
        rawShooterMotorHigh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.shooterHighMotor = new MotorCacher(rawShooterMotorHigh);
    }
    private void initHood() {
        ServoImplEx rawHoodLeftServo = hardwareMap.get(ServoImplEx.class, "hoodLeft");
        rawHoodLeftServo.setPwmRange(new PwmControl.PwmRange(hoodParams.downPWM, hoodParams.upPWM));
        this.hoodLeftServo = new ServoImplCacher(rawHoodLeftServo);

        ServoImplEx rawHoodRightServo = hardwareMap.get(ServoImplEx.class, "hoodRight");
        rawHoodRightServo.setPwmRange(new PwmControl.PwmRange(hoodParams.downPWM, hoodParams.upPWM));
        this.hoodRightServo = new ServoImplCacher(rawHoodRightServo);
    }
    private void initMisc() {
        prevLookAheads = new double[miscParams.lookAheadAvgNum];
        for(int i = 0; i < miscParams.lookAheadAvgNum; i++)
            prevLookAheads[i] = 0;
    }
    public void updateInfo(boolean useTurretLookAhead) {
        updateLookAheadTime(useTurretLookAhead);

        double prevTimeMs = curTimeMs;
        curTimeMs = System.currentTimeMillis();
        dt = (curTimeMs - prevTimeMs) / 1000;

        hoodLeftServo.updateProperties();
        hoodRightServo.updateProperties();

        Pose2d robotPose = robot.drive.localizer.getPose();
        futureRobotPose = robot.drive.pinpoint().getNextPoseSimple(currentLookAhead);
        isNear = robotPose.position.x < 24;
        targetPos = getGoalPos();

        updateTurretProperties(robotPose);
        shooterHighMotor.updateInfo();
        shooterLowMotor.updateInfo();

        rawShooterSpeedTps = (shooterHighMotor.getVelTps() + shooterLowMotor.getVelTps()) * 0.5;
        double a = miscParams.shooterTau == 0 ? 0 : Math.exp(-dt / miscParams.shooterTau);
        filteredShooterSpeedTps = filteredShooterSpeedTps * a + rawShooterSpeedTps * (1-a);

        Pose2d futureTurretPose = ShootingMath.getTurretPose(futureRobotPose, 0);
        Vector2d futureExitPos = ShootingMath.calculateExitPositionInches(futureTurretPose, ballExitAngleRad);
        double desiredBallDir = Math.atan2(targetPos.y - futureExitPos.y, targetPos.x - futureExitPos.x);

        Vector2d robotVel = robotVelAtExitPosIps.times(miscParams.enableShootingWhileMoving ? 0.0254 : 0);
        if(miscParams.usingLookup)
            updateLookupProperties(desiredBallDir, robotVel);
        else
            updatePhysicsProperties(desiredBallDir, robotVel);
    }

    // pro: yes velocity-based hood adjustment
    // con: math is weird and we have to do approximations
    private void updatePhysicsProperties(double desiredBallDir, Vector2d robotVel) {
        // finding target exit velocity
        double lookupDist = Range.clip(ballExitPosInchesFromGoal, lookupDistsI[0] + 0.01, lookupDistsI[lookupDistsI.length-1] - 0.01);
        ballTargetExitSpeedMps = lookupTable.lookupPhysicsVelocityMetersPerSec(lookupDist);
        efficiencyCoef = calcEfficiencyCoef(lookupTable.lookupExitAngleRad(lookupDist)); // initial guess for efficiency coef

        // get delta y of projectory (need approximate exit height of the ball)
        useHighArc = ballExitPosInchesFromGoal < mathParams.highArcDistThreshold;
        double exitHeightM = ShootingMath.approximateExitHeightM(useHighArc);
        double targetHeightIn;
        if (useHighArc) targetHeightIn = goalParams.highArcTargetHeightIn;
        else targetHeightIn = isNear ? goalParams.lowArcNearTargetHeightIn : goalParams.lowArcFarTargetHeightIn;
        relTargetHeightM = (targetHeightIn * 0.0254 - exitHeightM);

        // estimating hood ang and current shooter speed
        for(int i = 0; i < mathParams.numApproximations; i++) {
            curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
            physicsExitAngleRad = ShootingMath.calculateBallExitAngleRad(useHighArc, relTargetHeightM, ballExitPosInchesFromGoal, curExitSpeedMps);
            if(physicsExitAngleRad != -1)
                ballExitAngleRad = physicsExitAngleRad;
            efficiencyCoef = calcEfficiencyCoef(ballExitAngleRad);
        }
        curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);

        actualTargetExitSpeedMps = ballTargetExitSpeedMps;
        hoodExitAngleRad = ballExitAngleRad;
        absoluteTargetAngleRad = desiredBallDir;

        // allows for shooting while moving
//        actualTargetExitVelMps = ShootingMath.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, ballTargetExitSpeedMps, robotVel);


    }

    // pro: easy to tune
    // con: no velocity-based hood adjustment
    private void updateLookupProperties(double desiredBallDir, Vector2d robotVel) {
        // getting lookup properties
        double lookupDist = Range.clip(ballExitPosInchesFromGoal, lookupDistsI[0] + 0.01, lookupDistsI[lookupDistsI.length-1] - 0.01);
        ballExitAngleRad = lookupTable.lookupExitAngleRad(lookupDist);
        ballTargetExitSpeedMps = lookupTable.lookupVelocityMetersPerSec(lookupDist);

        // allows for shooting while moving
        actualTargetExitVelMps = ShootingMath.calculateActualTargetExitVel(desiredBallDir, ballExitAngleRad, ballTargetExitSpeedMps, robotVel);

        double baseLength = Math.hypot(actualTargetExitVelMps.x, actualTargetExitVelMps.z);
        hoodExitAngleRad = Math.atan2(actualTargetExitVelMps.y, baseLength);
        hoodExitAngleRad = Range.clip(hoodExitAngleRad, hoodParams.minExitAngRad, hoodParams.maxExitAngRad);
        absoluteTargetAngleRad = Math.atan2(actualTargetExitVelMps.z, actualTargetExitVelMps.x);

        efficiencyCoef = calcEfficiencyCoef(hoodExitAngleRad);
        curExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(filteredShooterSpeedTps, efficiencyCoef);
        actualTargetExitSpeedMps = Math.hypot( baseLength, actualTargetExitVelMps.y );
    }
    public void sendHardwareInfo() {
        turretMotor.sendInfo();
        shooterHighMotor.sendInfo();
        shooterLowMotor.sendInfo();
        if(Math.abs(hoodLeftServo.getPosition() - hoodLeftServo.getTargetPosition()) > hoodParams.resolution) {
            hoodLeftServo.sendInfo();
            hoodRightServo.sendInfo();
        }
    }
    private void updateTurretProperties(Pose2d robotPose) {
        turretMotor.updateInfo();
        turretPose = ShootingMath.getTurretPose(robotPose, robot.turret.currentRelativeAngleRad);
        ballExitPos = ShootingMath.calculateExitPositionInches(turretPose, ballExitAngleRad);
        odoVel = !robot.drive.pinpoint().previousVelocities.isEmpty() ? robot.drive.pinpoint().previousVelocities.get(0) : new OdoInfo(0, 0, 0);
        Vector2d robotVelCm = new Vector2d(odoVel.x, odoVel.y);
        Vector2d relativeExitPos = ballExitPos.minus(robotPose.position);
        Vector2d robotTanVel = new Vector2d(-relativeExitPos.y, relativeExitPos.x*1).times(odoVel.headingRad); // v = r * w
        robotVelAtExitPosIps = robotVelCm.plus(robotTanVel);
        robotSpeedAtExitPosIps = Math.hypot(robotVelAtExitPosIps.x, robotVelAtExitPosIps.y);
        robotSpeedAtExitPosIps = Math.hypot(robotVelAtExitPosIps.x, robotVelAtExitPosIps.y);

        double deltaX = targetPos.x - ballExitPos.x;
        double deltaY = targetPos.y - ballExitPos.y;
        ballExitPosInchesFromGoal = Math.hypot(deltaX, deltaY);
    }

    private void updateLookAheadTime(boolean useLookAhead) {
        double rawLookAhead = useLookAhead ? miscParams.rawLookAheadTime : 0;
        for(int i = prevLookAheads.length-1; i > 0; i--)
            prevLookAheads[i] = prevLookAheads[i-1];
        prevLookAheads[0] = rawLookAhead;
        currentLookAhead = getAvgLookAheads();
    }
    private double getAvgLookAheads() {
        double sum = 0;
        for (double prevLookAhead : prevLookAheads) sum += prevLookAhead;
        return sum / prevLookAheads.length;
    }
    private Vector2d getGoalPos() {
        if (BrainSTEMRobot.alliance == Alliance.RED) {
            if (robot.shootingSystem.isNear)
                return new Vector2d(goalParams.nearRedShotsGoalX, goalParams.nearRedShotsGoalY);
            return new Vector2d(goalParams.farRedShotsGoalX, goalParams.farRedShotsGoalY);
        }
        if (robot.shootingSystem.isNear)
            return new Vector2d(goalParams.nearBlueShotsGoalX, goalParams.nearBlueShotsGoalY);
        return new Vector2d(goalParams.farBlueShotsGoalX, goalParams.farBlueShotsGoalY);
    }

    public void printInfo(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addLine("SHOOTING SYSTEM-------");
        telemetry.addData("efficiency coef", efficiencyCoef);
        telemetry.addData("absolute turret target rad", absoluteTargetAngleRad);
        telemetry.addData("robot-relative target exit speed mps", actualTargetExitSpeedMps);
        telemetry.addData("ball exit angle rad", ballExitAngleRad);
        telemetry.addData("physics exit angle rad", physicsExitAngleRad);
        telemetry.addData("ball meters from goal", ballExitPosInchesFromGoal * 0.0254);
        telemetry.addData("ball inches from goal", ballExitPosInchesFromGoal);
        telemetry.addData("absolute target exit speed mps", ballTargetExitSpeedMps);
        telemetry.addData("dt", dt);
        telemetry.addLine();
        telemetry.addData("height to target meters", relTargetHeightM);
        telemetry.addData("high arc", useHighArc);
    }

    public int getTurretEncoder() {
        return turretMotor.getCurrentPosition();
    }
    public int getTurretEncoderRaw() {
        return turretMotor.getCurrentPositionRaw();
    }
    public void resetTurretEncoder() {
        turretMotor.resetEncoders();
    }
    public void setTurretPower(double p) {
        turretMotor.setPower(p);
    }
    public double getTurretPower() {
        return turretMotor.getPower();
    }
    public double getTurretVelTps() {
        return turretMotor.getVelTps();
    }
    public void setShooterPower(double p) {
        shooterHighMotor.setPower(p);
        shooterLowMotor.setPower(p);
    }
    public double calcEfficiencyCoef(double ballExitAngleRad) {
        return mathParams.efficiencyCoefM * ballExitAngleRad + mathParams.efficiencyCoefB;
    }
    public void setShooterPowerRaw(double p) {
        shooterHighMotor.setPowerRaw(p);
        shooterLowMotor.setPowerRaw(p);
    }
    public double getShooterPower() {
        return shooterHighMotor.getPower();
    }
    public double getShooterHighVelTps() {
        return shooterHighMotor.getVelTps();
    }
    public double getShooterLowVelTps() {
        return shooterLowMotor.getVelTps();
    }
    public double getPrevShooterVelTps() {
        return (shooterHighMotor.getPrevVelTps() + shooterLowMotor.getPrevVelTps()) * 0.5;
    }

    public void setHoodPosition(double p) {
        hoodLeftServo.setPosition(p);
        hoodRightServo.setPosition(p);
    }
    public double getHoodPosition() {
        return hoodLeftServo.getPosition();
    }
}
