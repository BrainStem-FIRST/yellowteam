package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.Component;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.PIDController;

@Config
public final class Turret implements Component {
    public boolean isRedAlliance = true;
    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx turretMotor;
    private PIDController pidController;

    public TurretState turretState;
    Pose2d targetPose = new Pose2d(72, 72, 0);

    public static class Params{
        public double kP = 0.01;
        public double kI = 0;
        public double kD = 0;
        public int TURRET_INCREMENT = 60;
        public int TURRET_MAX = 300;
        public int TURRET_MIN = -300;
        public int TICKS_PER_REV = -1680;
        public int RIGHT_BOUND = -300;
        public int LEFT_BOUND = 300;
    }
    public MecanumDrive drive;
    public static Params TURRET_PARAMS = new Turret.Params();

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, MecanumDrive drive){
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.drive = drive;

        turretMotor = map.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController = new PIDController(TURRET_PARAMS.kP, TURRET_PARAMS.kI, TURRET_PARAMS.kD);
        turretState = TurretState.OFF;
    }


    public int getTurretEncoder() {
        return turretMotor.getCurrentPosition();
    }

    public void setTurretPosition(int ticks) {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double error = getTurretEncoder() - ticks;
        double power = pidController.updateWithError(error);
        telemetry.addData("Turret Power", power);
        turretMotor.setPower(-power);
    }

    public void pointTurretAtTarget(Pose2d robotPose, Pose2d targetPose) {
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double turretMax = Math.toRadians(90);
        double turretMin = Math.toRadians(-90);

        double targetAngle = Math.atan2(deltaY, deltaX);
        double turretTargetAngle = targetAngle - robotPose.heading.toDouble();
        turretTargetAngle = Math.atan2(Math.sin(turretTargetAngle), Math.cos(turretTargetAngle));

        // COMMENT OUT IF NOT WORKING //
        if (turretTargetAngle > turretMax)
            turretTargetAngle = Math.toRadians(180) - turretTargetAngle; // mirror
        else if (turretTargetAngle < turretMin)
            turretTargetAngle = -Math.toRadians(180) - turretTargetAngle; // mirror

        double turretTicksPerRadian = TURRET_PARAMS.TICKS_PER_REV / (2 * Math.PI);
        int targetTurretPosition = (int)(-turretTargetAngle * turretTicksPerRadian);

//        if (targetTurretPosition > TURRET_PARAMS.RIGHT_BOUND && targetTurretPosition < TURRET_PARAMS.LEFT_BOUND)
//            setTurretPosition(targetTurretPosition);
//        else
//            setTurretPosition(0);

        targetTurretPosition = Math.max(TURRET_PARAMS.RIGHT_BOUND, Math.min(targetTurretPosition, TURRET_PARAMS.LEFT_BOUND));
        setTurretPosition(targetTurretPosition);
    }

    public enum TurretState {
        OFF, TRACKING, CENTER
    }

    @Override
    public void reset() {
    }

    @Override
    public void update(){
        switch (turretState) {
            case OFF: {
//                turretMotor.setPower(0);
                break;
            } case TRACKING: {
                pointTurretAtTarget(drive.localizer.getPose(), targetPose);
                break;
            } case CENTER: {
                setTurretPosition(0);
                break;
            }
        }

        if (isRedAlliance)
            targetPose = new Pose2d(72, 72, 0);
        else
            targetPose = new Pose2d(72, -72, 0);

        telemetry.addData("Alliance", isRedAlliance ? "Red" : "Blue");
        telemetry.addData("Turret State", turretState.toString());
        telemetry.addData("Turret Encoder", getTurretEncoder());
    }
    @Override
    public String test(){
        return null;
    }
}
