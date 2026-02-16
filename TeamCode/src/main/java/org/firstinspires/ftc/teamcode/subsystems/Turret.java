package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Turret extends Component {
    public static class Params {
        public double offsetFromCenter = 3.442; // offset of center of turret from center of robot in inches

        public int fineAdjust = 5;
        public double TICKS_PER_REV = 1228.5, ticksPerRad = TICKS_PER_REV / (2 * Math.PI);
        public int minEncoderBound = -300;
        public int maxEncoderBound = 300;
        public double minAngle = Math.toRadians(-90);
        public double maxAngle = Math.toRadians(90);
    }
    public static class PowerTuning {
        public double A = .01, k = .02, x0 = 150;
        public double kV = 0.0003, kVP = 0.001;
        public double decelTime = .2;
        public double[] kfLookupEncoders = new double[] {-350, -1, 0, 1, 350};
        public double[] kfLookupPowers = new double[] {0, 0, 0, 0, 0};
    }
    public static Params turretParams = new Params();
    public static PowerTuning powerTuning = new PowerTuning();
    private double targetEncoder, targetVelocity, targetAngularVelocity;
    private double firstTimeWhereTargetVelIsZero;

    private double currentEncoder, currentVelocity;
    private double positionError, velocityError;
    private double kP, kF, dir;
    private final InterpLUT kFLookup;

    private double targetAngle;

    private double currentRelativeAngleRad;
    private boolean inRange;
    private final DcMotorEx turretMotor;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry){
        super(hardwareMap, telemetry);
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        kFLookup = new InterpLUT();
        for(int i = 0; i < powerTuning.kfLookupEncoders.length; i++)
            kFLookup.add(powerTuning.kfLookupEncoders[i], powerTuning.kfLookupPowers[i]);
        kFLookup.createLUT();
        updateProperties();
    }
    public void updateProperties() {
        currentEncoder = turretMotor.getCurrentPosition();
        currentVelocity = turretMotor.getVelocity();
        currentRelativeAngleRad = currentEncoder / turretParams.ticksPerRad;
    }
    public double calculateTurretPower() {
        positionError = targetEncoder - currentEncoder;
        double timeSinceTargetVelFirstZero = (System.currentTimeMillis() - firstTimeWhereTargetVelIsZero) / 1000;
        velocityError = targetVelocity == 0 && timeSinceTargetVelFirstZero > powerTuning.decelTime ? 0 : targetVelocity - currentVelocity;
        dir = Math.signum(positionError);
        kP = calcLogisticKP(Math.abs(positionError));
        double input = currentEncoder * dir; // reversing input if traveling in the opposite direction
        kF = kFLookup.get(input) * dir;
        return kP * positionError + kF + powerTuning.kV * targetVelocity + powerTuning.kVP * velocityError;
    }
    private double calcLogisticKP(double errorMag) {
        return powerTuning.A / (1 + Math.exp(-powerTuning.k * (errorMag - powerTuning.x0)));
    }
    public void setTarget(double relativeTargetAngle, double targetAngularVelocity) {
        // updating position variables
        this.targetAngle = relativeTargetAngle;
        // mirrors the angle if the turret cannot reach it (visual cue)
        if (this.targetAngle > Math.toRadians(90)) {
            this.targetAngle = Math.PI - this.targetAngle;
            inRange = false;
        }
        else if (this.targetAngle < Math.toRadians(-90)) {
            this.targetAngle = -Math.PI - this.targetAngle;
            inRange = false;
        }
        else
            inRange = true;

        targetEncoder = this.targetAngle * turretParams.ticksPerRad;
        this.targetAngularVelocity = targetAngularVelocity;
        double prevTargetVelocity = targetVelocity;
        targetVelocity = targetAngularVelocity * turretParams.ticksPerRad;
        if(targetVelocity == 0 && prevTargetVelocity != 0)
            firstTimeWhereTargetVelIsZero = System.currentTimeMillis();
    }
    public void setPower(double power) {
        turretMotor.setPower(power);
    }

    @Override
    public void printInfo() {
        double turretTicksPerDegree = turretParams.TICKS_PER_REV / 360.;

        telemetry.addLine("TURRET------");
        telemetry.addLine("-----");
        telemetry.addData("turret power", turretMotor.getPower());
        telemetry.addData("kP", kP);
        telemetry.addData("kf", kF);
        telemetry.addLine("-----");
        telemetry.addData("target encoder", targetEncoder);
        telemetry.addData("target velocity", targetVelocity);
        telemetry.addData("target angular velocity", targetAngularVelocity);
        telemetry.addLine("-----");
        telemetry.addData("current encoder", currentEncoder);
        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("turret current relative angle deg", Math.toDegrees(currentRelativeAngleRad));
        telemetry.addData("turret target relative angle deg", Math.toDegrees(targetAngle));
        telemetry.addData("dir", dir);
        telemetry.addLine("-----");
        telemetry.addData("angle degree error", positionError / turretTicksPerDegree);
        telemetry.addData("encoder error", positionError);
        telemetry.addData("velocity error", velocityError);
        telemetry.addLine("-----");
        telemetry.addData("inRange", inRange());
    }
    public boolean inRange() {
        return inRange;
    }
    public double getRelAngleRad() {
        return currentRelativeAngleRad;
    }
    public double getAbsAngleRad(double robotHeading) {
        return robotHeading + currentRelativeAngleRad;
    }
    public double ticksToAngle(double ticks) {
        return ticks / turretParams.ticksPerRad;
    }
    public void resetEncoders() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getEncoder() {
        return currentEncoder;
    }
    public double getVelocity() {
        return currentVelocity;
    }
}
