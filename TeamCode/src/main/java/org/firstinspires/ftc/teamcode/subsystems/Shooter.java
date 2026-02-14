package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;

@Config
public class Shooter extends Component {
    public static class ShooterParams {
        public double kP = 0.005;
        public double kI = 0.0;
        public double kD = 0.0;
        public double kF = 0.00045;
        public double tolerance = 40;
        public double minPower = -0.15, maxPower = 0.99;
        public double shotRecoveryPower = 0.99, shotRecoveryError = 40;
        public double shooterTau = .2;
        public double fineAdjust = 10;
    }
    public static class HoodParams {
        public double downPWM = 900, upPWM = 2065;
    }

    public static class TestingParams {
        public boolean testing = false;
        public double testingVel = 1500;
        public double testingExitAngleRad = 1.0472;
    }

    public static ShooterParams shooterParams = new ShooterParams();
    public static HoodParams hoodParams = new HoodParams();
    public static TestingParams testingParams = new TestingParams();


    private final PIDController shooterPID;

    private final DcMotorEx shooterLowMotor, shooterHighMotor;
    private double shooterLowSpeed, shooterHighSpeed;
    private double filteredShooterSpeed, rawShooterSpeed;
    private double shooterPower;

    private final ServoImplEx hoodLeftServo, hoodRightServo;
    private double hoodPosition;

    private int ballsShot;
    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        shooterLowMotor = hardwareMap.get(DcMotorEx.class, "lowShoot");
        shooterLowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterLowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterHighMotor = hardwareMap.get(DcMotorEx.class, "highShoot");
        shooterHighMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterHighMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterHighMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterHighMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hoodLeftServo = hardwareMap.get(ServoImplEx.class, "hoodLeft");
        hoodLeftServo.setPwmRange(new PwmControl.PwmRange(hoodParams.downPWM, hoodParams.upPWM));

        hoodRightServo = hardwareMap.get(ServoImplEx.class, "hoodRight");
        hoodRightServo.setPwmRange(new PwmControl.PwmRange(hoodParams.downPWM, hoodParams.upPWM));

        shooterPID = new PIDController(shooterParams.kP, shooterParams.kI, shooterParams.kD);

        updateProperties(0);
    }
    public void setShooterVelocityPID(double targetVelocityTicksPerSec, double currentShooterVelocity) {
        shooterPID.setTarget(targetVelocityTicksPerSec);
        double pidOutput = -shooterPID.update(currentShooterVelocity);
        double feedForward = shooterParams.kF * targetVelocityTicksPerSec;
        double totalPower = pidOutput + feedForward;

        totalPower = Range.clip(totalPower, shooterParams.minPower, shooterParams.maxPower);
        double error = targetVelocityTicksPerSec - currentShooterVelocity;
        if(error > shooterParams.shotRecoveryError)
            totalPower = shooterParams.shotRecoveryPower;

        setShooterPower(totalPower);
    }

    public void updateProperties(double dt) {
        shooterHighSpeed = shooterHighMotor.getVelocity();
        shooterLowSpeed = shooterLowMotor.getVelocity();
        rawShooterSpeed = (shooterHighSpeed + shooterLowSpeed) * .5;
        double a = shooterParams.shooterTau == 0 ? 0 : Math.exp(-dt / shooterParams.shooterTau);
        filteredShooterSpeed = filteredShooterSpeed * a + rawShooterSpeed * (1-a);
        shooterPower = shooterHighMotor.getPower();

        hoodPosition = hoodLeftServo.getPosition();
    }
    public void updateTarget(double targetShooterSpeedTps, double targetExitAngRad) {
        if(testingParams.testing)
            setShooterVelocityPID(testingParams.testingVel, filteredShooterSpeed);
        else
            setShooterVelocityPID(targetShooterSpeedTps, filteredShooterSpeed);

        double pos = ShootingMath.getHoodServoPosition(testingParams.testing ? testingParams.testingExitAngleRad : targetExitAngRad);
        setHoodPosition(pos);
    }
    @Override
    public void printInfo() {
        telemetry.addLine("SHOOTER------");
        telemetry.addData("  pid target vel", shooterPID.getTarget());
        telemetry.addData("  shooter power", shooterPower);
        telemetry.addData("  shooter filtered vel tps", filteredShooterSpeed);
        telemetry.addData("  shooter raw vel tps", rawShooterSpeed);
        telemetry.addData("  high motor vel", shooterHighSpeed);
        telemetry.addData("  low motor vel", shooterLowSpeed);

        telemetry.addLine();
        telemetry.addLine("HOOD------");
        telemetry.addData("  hood pos", hoodPosition);
    }
    public boolean inTolerance(double targetSpeedTps) {
        return (targetSpeedTps - filteredShooterSpeed) < shooterParams.tolerance;
    }
    public void setShooterPower(double power) {
        shooterHighMotor.setPower(power);
        shooterLowMotor.setPower(power);
    }

    public void setHoodPosition(double pos) {
        hoodLeftServo.setPosition(pos);
        hoodRightServo.setPosition(pos);
    }

    public double getFilteredShooterSpeed() {
        return filteredShooterSpeed;
    }
    public double getHoodPosition() {
        return hoodPosition;
    }
    public double getPidError() {
        return shooterPID.getTarget() - filteredShooterSpeed;
    }
    public double getPower() {
        return shooterPower;
    }

    public int getBallsShot() {
        return ballsShot; // TODO: re add ball tracking logic
    }
    public void setBallsShot(int num) {
        ballsShot = num;
    }
}
