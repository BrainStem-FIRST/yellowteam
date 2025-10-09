package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;

@Config
public class Collection implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    public DcMotorEx collectorMotor;
    public ServoImplEx shifter;
    public CollectionState collectionState;

    public static class Params{
        private double LOCK_POSITION = 0;
        private double UNLOCK_POSITION = 1;
    }

    public static Params COLLECTOR_PARAMS = new Collection.Params();

    public Collection(HardwareMap hardwareMap, Telemetry telemetry){
        this.map = hardwareMap;
        this.telemetry = telemetry;

//        collectorMotor = map.get(DcMotorEx.class, "collection");
//        shifter = map.get(ServoImplEx.class, "shifter");
        collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void transferToShooter() {
        shifter.setPosition(COLLECTOR_PARAMS.LOCK_POSITION);
        collectorMotor.setPower(0.1);
    }
    public enum CollectionState {
        OFF, INTAKE, EXTAKE, TRANSFER
    }

    @Override
    public void reset() {
    }

    @Override
    public void update() {
//        telemetry.addData("Turret State", turretState.toString());
        telemetry.update();
        switch (collectionState) {
            case OFF: {
                collectorMotor.setPower(0);
                shifter.setPosition(COLLECTOR_PARAMS.UNLOCK_POSITION);
                break;
            }
            case INTAKE: {
                collectorMotor.setPower(0.5);
                break;
            }
            case EXTAKE: {
                collectorMotor.setPower(-0.5);
                break;
            }
            case TRANSFER: {
                transferToShooter();
                break;
            }
        }
    }

    @Override
    public String test(){
        return null;
    }
}
