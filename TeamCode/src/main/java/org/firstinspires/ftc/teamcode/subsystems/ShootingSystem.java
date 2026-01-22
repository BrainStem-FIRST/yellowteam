package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterLookup.dists;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.utils.misc.MotorCacher;
import org.firstinspires.ftc.teamcode.utils.misc.ServoImplCacher;

public class ShootingSystem {
    public static class GoalParams {
        public double nearRedShotsGoalX = -65, nearRedShotsGoalY = 65, farRedShotsGoalX = -66, farRedShotsGoalY = 65;
        public double nearBlueShotsGoalX = -66, nearBlueShotsGoalY = -65, farBlueShotsGoalX = -67, farBlueShotsGoalY = -63;
    }
    public static GoalParams goalParams = new GoalParams();
    public static class HoodParams {
        public double downPWM = 900, upPWM = 2065;
    }
    public static class MiscParams {
        // efficiency coef regression: y=-0.0766393x+0.446492
        public double efficiencyCoefM = -0.0766393, efficiencyCoefB = 0.446492;
        public int lookAheadAvgNum = 5;
        public double rawLookAheadTime = 0.2; // time to look ahead for pose prediction
    }
    public static MiscParams miscParams = new MiscParams();
    public static HoodParams hoodParams = new HoodParams();
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
    public Vector2d actualTargetBallExitVelMps;
    public double actualTargetBallExitSpeedMps;
    public double absoluteTargetExitSpeedMps;
    public double efficiencyCoef;

    public double curExitVelMps;
    public double ballExitAngleRad;
    public Vector2d ballExitPos, prevBallExitPos;
    public double ballExitPosInchesFromGoal;
    public Pose2d turretPose;

    private double curTimeMs;
    public double dt;

    public ShooterLookup lookupTable;
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
        Pose2d futureTurretPose = ShootingMath.getTurretPose(futureRobotPose, 0);

        double lookupDist = Range.clip(ballExitPosInchesFromGoal, dists[0] + 0.01, dists[dists.length-1] - 0.01);
        ballExitAngleRad = lookupTable.lookupExitAngleRad(lookupDist);
        efficiencyCoef = miscParams.efficiencyCoefM * ballExitAngleRad + miscParams.efficiencyCoefB;

        shooterHighMotor.updateInfo();
        shooterLowMotor.updateInfo();

        curExitVelMps = ShootingMath.ticksPerSecToExitSpeedMps(getShooterVelTps(), efficiencyCoef);

        absoluteTargetExitSpeedMps = ShootingMath.ticksPerSecToExitSpeedMps(lookupTable.lookupVelocityTicksPerSec(lookupDist), efficiencyCoef);
        actualTargetBallExitVelMps = ShootingMath.calculateActualTargetExitVel(targetPos, futureTurretPose.position, absoluteTargetExitSpeedMps, robotVelAtExitPosIps);
        actualTargetBallExitSpeedMps = Math.hypot(actualTargetBallExitVelMps.x, actualTargetBallExitVelMps.y);

        absoluteTargetAngleRad = Math.atan2(actualTargetBallExitVelMps.y, actualTargetBallExitVelMps.x);
    }
    public void sendHardwareInfo() {
        turretMotor.sendInfo();
        shooterHighMotor.sendInfo();
        shooterLowMotor.sendInfo();
    }
    private void updateTurretProperties(Pose2d robotPose) {
        turretMotor.updateInfo();
        turretPose = ShootingMath.getTurretPose(robotPose, robot.turret.currentRelativeAngleRad);
        prevBallExitPos = ballExitPos;
        ballExitPos = ShootingMath.calculateExitPositionInches(turretPose, ballExitAngleRad);
        robotVelAtExitPosIps = ballExitPos.minus(prevBallExitPos);
        robotVelAtExitPosIps = robotVelAtExitPosIps.div(dt);
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
        for(int i = 0; i < prevLookAheads.length; i++)
            sum += prevLookAheads[i];
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
    public void setShooterPowerRaw(double p) {
        shooterHighMotor.setPowerRaw(p);
        shooterLowMotor.setPowerRaw(p);
    }
    public double getShooterPower() {
        return shooterHighMotor.getPower();
    }
    public double getShooterVelTps() {
        return (shooterHighMotor.getVelTps() + shooterLowMotor.getVelTps()) * 0.5;
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
