package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Parking extends Component {
    public ServoImplEx parkLeftServo;
    public ServoImplEx parkRightServo;
    public ParkState parkState;

    public static class Params{
        public double RETRACTED_POS = 0.075;
        public double EXTENDED_POS = 0.9;
        public double MIDDLE_POS = 0.5;
        public double SERVO_INCREMENT = 0.1;
//        public double SCALE_FACTOR = 255;
    }

    public static Params PARK_PARAMS = new Parking.Params();

    public Parking(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        parkLeftServo = hardwareMap.get(ServoImplEx.class, "parkLeft");
        parkLeftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        parkRightServo = hardwareMap.get(ServoImplEx.class, "parkRight");
        parkRightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        parkState = ParkState.RETRACTED;
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
    public void printInfo() {}

    @Override
    public void reset() {}

    @Override
    public void update() {
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
        }
    }
}
