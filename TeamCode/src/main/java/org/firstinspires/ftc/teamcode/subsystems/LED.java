package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LED extends Component {
    private final ServoImplEx left_led;
    private final ServoImplEx right_led;
    public LED(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        right_led = hardwareMap.get(ServoImplEx.class, "rightLED");
        left_led = hardwareMap.get(ServoImplEx.class, "leftLED");
    }

    @Override
    public void printInfo() {}

    @Override
    public void reset() {}

    @Override
    public void update(){

        if (robot.collection.clutchState == Collection.ClutchState.ENGAGED) {
            left_led.setPosition(0.45); //green
            right_led.setPosition(0.45);
        } else if (robot.collection.clutchState == Collection.ClutchState.UNENGAGED) {
            if (robot.collection.intakeHas3Balls()) {
                left_led.setPosition(0.666); //blue
                right_led.setPosition(0.666);
            } else {
                left_led.setPosition(0.279); //red
                right_led.setPosition(0.279);
            }
        }
    }
}
