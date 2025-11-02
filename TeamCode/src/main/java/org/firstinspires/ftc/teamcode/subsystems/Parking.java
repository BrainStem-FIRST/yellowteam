package org.firstinspires.ftc.teamcode.subsystems;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
public class Parking implements Component {
    private HardwareMap map;
    private Telemetry telemetry;
    private MecanumDrive drive;
    public ServoImplEx parkLeftServo;
    public ServoImplEx parkRightServo;
//    public ColorSensor leftColorSensor;
//    public ColorSensor rightColorSensor;
    public ParkState parkState;

    public static class Params{
        public double RETRACTED_POS = 0.01;
        public double EXTENDED_POS = 0.99;
        public double MIDDLE_POS = 0.5;
        public double SERVO_INCREMENT = 0.1;
//        public double SCALE_FACTOR = 255;
    }

    public static Params PARK_PARAMS = new Parking.Params();

    public Parking(HardwareMap hardwareMap, Telemetry telemetry, MecanumDrive drive){
        this.map = hardwareMap;
        this.telemetry = telemetry;
        this.drive = drive;

        parkLeftServo = map.get(ServoImplEx.class, "parkLeft");
        parkLeftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        parkRightServo = map.get(ServoImplEx.class, "parkRight");
        parkRightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

//        leftColorSensor = hardwareMap.get(ColorSensor.class, "leftColor");
//        rightColorSensor = hardwareMap.get(ColorSensor.class, "rightColor");

        parkState = ParkState.OFF;
    }

//    public double getColorSensorBrightness() {
//        float[] hsv = new float[3];
//        Color.RGBToHSV((int) (leftColorSensor.red() * PARK_PARAMS.SCALE_FACTOR),
//                (int) (leftColorSensor.green() * PARK_PARAMS.SCALE_FACTOR),
//                (int) (leftColorSensor.blue() * PARK_PARAMS.SCALE_FACTOR),
//                hsv);
//        return hsv[2];
//    }

    public void setParkServoPosition(double position) {
        parkLeftServo.setPosition(position);
        parkRightServo.setPosition(position);
    }

    public enum ParkState {
        RETRACTED, EXTENDED, MIDDLE, OFF
    }

    @Override
    public void reset() {}

    @Override
    public void update(){
        switch (parkState) {
            case RETRACTED:
                setParkServoPosition(PARK_PARAMS.RETRACTED_POS);
                break;

            case EXTENDED:
                setParkServoPosition(PARK_PARAMS.EXTENDED_POS);
                break;

            case MIDDLE:
                setParkServoPosition(PARK_PARAMS.MIDDLE_POS);
                break;

            case OFF:
                break;
        }
    }
    @Override
    public String test(){
        return null;
    }
}
