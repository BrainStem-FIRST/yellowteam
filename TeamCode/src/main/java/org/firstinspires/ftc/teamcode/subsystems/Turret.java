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

    private HardwareMap map;
    private Telemetry telemetry;
    private DcMotorEx turretMotor;
    private PIDController pidController;

    private int RIGHT_BOUND = -300;
    private int LEFT_BOUND = 300;

    public TurretState turretState;
    Pose2d targetPose = new Pose2d(72, 72, 0);

    public static class Params{
        public double kP = 0.01;
        public double kI = 0;
        public double kD = 0;

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
//        pidController.setInputBounds(-300, 300);
//        pidController.setOutputBounds(1, -1);
        turretState = TurretState.OFF;
    }



    public int getTurretEncoder() {
        return turretMotor.getCurrentPosition();
    }

    public void setTurretPosition(int ticks) {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double error = getTurretEncoder() - ticks;
        double power = pidController.updateWithError(error);
        telemetry.addData("power", -power);
        telemetry.addData("Turret Encoder", getTurretEncoder());
        telemetry.update();
        turretMotor.setPower(-power);
    }



    public void pointTurretAtTarget(Pose2d robotPose, Pose2d targetPose) {
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        telemetry.addData("Pose X", robotPose.position.x);
        telemetry.addData("Pose Y", robotPose.position.y);
        telemetry.addData("Pose Heading", Math.toDegrees(robotPose.heading.toDouble()));


        double targetAngle = Math.atan2(deltaY, deltaX);
        double turretTargetAngle = targetAngle - robotPose.heading.toDouble();
        turretTargetAngle = Math.atan2(Math.sin(turretTargetAngle), Math.cos(turretTargetAngle));
        telemetry.addData("Turret Angle", turretTargetAngle);

        double ticksPerRev = -1680;
        double turretTicksPerRadian = ticksPerRev / (2 * Math.PI);
        int targetTurretPosition = (int)(-turretTargetAngle * turretTicksPerRadian);
        telemetry.addData("Turret Ticks", targetTurretPosition);
        telemetry.addData("Turret Encoder", getTurretEncoder());
        telemetry.update();

        if (targetTurretPosition > -350 && targetTurretPosition < 350)
            setTurretPosition(targetTurretPosition);
        else{
            setTurretPosition(0);
        }



    }




    public enum TurretState {
        OFF, TRACKING, CENTER
    }

    public void setTurretTracking(){
        turretState = TurretState.TRACKING;
    }

    public void setTurretOff(){
        turretState = TurretState.OFF;
    }
    public void setTurretCenter(){
        turretState = TurretState.CENTER;
    }




    @Override
    public void reset() {
    }

    @Override
    public void update(){
        switch (turretState) {
            case OFF: {
                turretMotor.setPower(0);
                break;
            } case TRACKING: {
                pointTurretAtTarget(drive.localizer.getPose(), targetPose);
                break;
            } case CENTER: {
                setTurretPosition(0);
                break;
            }
        }
    }
    @Override
    public String test(){
        return null;
    }
}
