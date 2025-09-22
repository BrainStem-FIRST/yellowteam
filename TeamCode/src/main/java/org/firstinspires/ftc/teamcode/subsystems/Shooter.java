package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Component;
import org.firstinspires.ftc.teamcode.utils.PIDController;

public class Shooter implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx shooterMotor;

    public static class Params{

    }

    public static Params SHOOTER_PARAMS = new Shooter.Params();

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry){
        this.map = hardwareMap;
        this.telemetry = telemetry;

        shooterMotor = map.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void reset() {
    }

    @Override
    public void update(){
        telemetry.addData("Shooter Current", shooterMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
    @Override
    public String test(){
        return null;
    }
}
