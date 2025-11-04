package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;

@Config
public class Collection implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    private DcMotorEx collectorMotor;
    private ServoImplEx clutchLeft;
    private ServoImplEx clutchRight;

    //Swyft Sensors (SET BOTH DIP SWITCHES TO 0)

    private AnalogInput frontRightLaser;
    private AnalogInput frontLeftLaser;
    private AnalogInput backTopLaser;
    private AnalogInput backBottomLaser;

    public CollectionState collectionState;
    public ClutchState clutchState;

    private double timerStart = 0;
    private boolean timerRunning = false;
    private boolean has3Balls = false;
    private final ElapsedTime timer = new ElapsedTime();

    public static class Params{
        public double ENGAGED_POS = 0.1;
        public double DISENGAGED_POS = 0.95;
        public double DELAY_PERIOD = 2;
        public double INTAKE_SPEED = 1.0;
        public double LASER_BALL_THRESHOLD = 20;
    }

    public static Params COLLECTOR_PARAMS = new Collection.Params();

    public Collection(HardwareMap hardwareMap, Telemetry telemetry){
        this.map = hardwareMap;
        this.telemetry = telemetry;

        collectorMotor = map.get(DcMotorEx.class, "intake");
        collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clutchRight = map.get(ServoImplEx.class, "clutchRight");
        clutchRight.setPwmRange(new PwmControl.PwmRange(1450, 2000));

        clutchLeft = map.get(ServoImplEx.class, "clutchLeft");
        clutchLeft.setPwmRange(new PwmControl.PwmRange(1450, 2000));

        frontRightLaser = hardwareMap.get(AnalogInput.class, "FRLaser");
        frontLeftLaser = hardwareMap.get(AnalogInput.class, "FLLaser");
        backTopLaser = hardwareMap.get(AnalogInput.class, "BRLaser");
        backBottomLaser = hardwareMap.get(AnalogInput.class, "BLLaser");

        collectionState = CollectionState.OFF;
        clutchState = ClutchState.UNENGAGED;

        timer.reset();
    }

    private double voltageToDistance(double voltage) {
        return (voltage * 48.78136376) - 4.985354503; //tune if not accurate
    }

    private boolean isBackBallDetected() {
        return (voltageToDistance(backBottomLaser.getVoltage())) < COLLECTOR_PARAMS.LASER_BALL_THRESHOLD ||
                (voltageToDistance(backTopLaser.getVoltage())) < COLLECTOR_PARAMS.LASER_BALL_THRESHOLD;
    }

    private boolean isFrontBallDetected() {
        return (voltageToDistance(frontRightLaser.getVoltage())) < COLLECTOR_PARAMS.LASER_BALL_THRESHOLD ||
                (voltageToDistance(frontLeftLaser.getVoltage())) < COLLECTOR_PARAMS.LASER_BALL_THRESHOLD;
    }

    public void startIntake() {
        collectionState = CollectionState.INTAKE;
        timerStart = 0;
        timerRunning = false;
    }

    public void stopIntake() {
        collectionState = CollectionState.OFF;
        timerRunning = false;
    }

    public boolean intakeHas3Balls() {
        return has3Balls;
    }

    public void checkForIntakeBalls(double currentTime) {
        if (isBackBallDetected() && isFrontBallDetected()) {
            if (!timerRunning) {
                timerStart = currentTime;
                timerRunning = true;
            } else if (currentTime - timerStart > COLLECTOR_PARAMS.DELAY_PERIOD) {
                has3Balls = true;
            }
        } else {
            timerRunning = false;
            timerStart = 0;
            has3Balls = false;
        }
    }

    public enum CollectionState {
        OFF, INTAKE, EXTAKE, TRANSFER
    }

    public enum ClutchState {
        ENGAGED, UNENGAGED
    }

    @Override
    public void reset() {
    }

    @Override
    public void update() {

        switch (collectionState) {
            case OFF:
                collectorMotor.setPower(0);
                break;

            case INTAKE:
                collectorMotor.setPower(COLLECTOR_PARAMS.INTAKE_SPEED);
                break;

            case EXTAKE:
                collectorMotor.setPower(-COLLECTOR_PARAMS.INTAKE_SPEED);
                break;

            case TRANSFER:
                collectorMotor.setPower(0.1);
                break;
        }

        switch (clutchState) {
            case ENGAGED:
                clutchRight.setPosition(COLLECTOR_PARAMS.ENGAGED_POS);
                clutchLeft.setPosition(COLLECTOR_PARAMS.ENGAGED_POS);
                break;

            case UNENGAGED:
                clutchRight.setPosition(COLLECTOR_PARAMS.DISENGAGED_POS);
                clutchLeft.setPosition(COLLECTOR_PARAMS.DISENGAGED_POS);
                break;
        }

        checkForIntakeBalls(timer.seconds());

        telemetry.addData("Laser Distance", voltageToDistance(backBottomLaser.getVoltage()));
        telemetry.addData("Back Ball Detected", isBackBallDetected());
        telemetry.addData("Front Ball Detected", isFrontBallDetected());
        telemetry.addData("Collection State", collectionState.toString());
        telemetry.addData("Clutch State", clutchState.toString());
    }

    @Override
    public String test(){
        return null;
    }
}
