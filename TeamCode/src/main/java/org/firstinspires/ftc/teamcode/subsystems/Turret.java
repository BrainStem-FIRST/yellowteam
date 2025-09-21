package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;

public class Turret implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    private DcMotorEx turretMotor;

    public static class Params{

    }

    public Turret(HardwareMap hardwareMap, Telemetry telemetry){
        this.map = hardwareMap;
        this.telemetry = telemetry;

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
    }

    public int getTurretEncoder() {
        return turretMotor.getCurrentPosition();
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
