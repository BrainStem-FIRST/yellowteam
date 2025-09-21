package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Turret implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    private DcMotorEx turretMotor;
    private PIDController pidController;

    private int RIGHT_BOUND = -300;
    private int LEFT_BOUND = 300;

    public static class Params{
        public double kP = 0.001;
        public double kI = 0;
        public double kD = 0;

    }

    public static Params PARAMS = new Turret.Params();

    public Turret(HardwareMap hardwareMap, Telemetry telemetry){
        this.map = hardwareMap;
        this.telemetry = telemetry;

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);
//        pidController.setInputBounds(-300, 300);
//        pidController.setOutputBounds(1, -1);
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

//    public double getTurretPower(Pose2d robotPose, Pose2d targetPose) {
//        double dx = targetPose.position.x - robotPose.position.x;
//        double dy = targetPose.position.y - robotPose.position.y;
//        double targetHeading = Math.atan2(dy, dx);
//
//        double turretHeading = robotPose.heading.toDouble()
//                + (getTurretEncoder() * (2 * Math.PI / TICKS_PER_REV));
//
//        double error = normalizeAngle(targetHeading - turretHeading);
//
//        return pidController.updateWithError(error);
//    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    @Override
    public void reset() {
    }

    @Override
    public void update(){
    }
    @Override
    public String test(){

        return null;
    }
}
