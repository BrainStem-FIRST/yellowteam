package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
public class LED implements Component {
    private HardwareMap map;
    private Telemetry telemetry;
    private Shooter shooter;
    private Turret turret;
    private Parking parking;
    private Collection collection;
    private ServoImplEx left_led;
    private ServoImplEx right_led;
    public LED(HardwareMap hardwareMap, Telemetry telemetry, Shooter shooter, Turret turret, Parking parking, Collection collection){
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.shooter = shooter;
        this.turret = turret;
        this.parking = parking;
        this.collection = collection;

        right_led = hardwareMap.get(ServoImplEx.class, "rightLED");
        left_led = hardwareMap.get(ServoImplEx.class, "leftLED");
    }

    @Override
    public void reset() {}

    @Override
    public void update(){

        if (collection.clutchState == Collection.ClutchState.ENGAGED) {
            left_led.setPosition(0.37); //green
            right_led.setPosition(0.37);
        } else if (collection.clutchState == Collection.ClutchState.UNENGAGED) {
            if (collection.intakeHas3Balls()) {
                left_led.setPosition(0.666); //blue
                right_led.setPosition(0.666);
            } else {
                left_led.setPosition(0.279); //red
                right_led.setPosition(0.279);
            }
        }
    }

    @Override
    public String test(){
        return null;
    }
}
