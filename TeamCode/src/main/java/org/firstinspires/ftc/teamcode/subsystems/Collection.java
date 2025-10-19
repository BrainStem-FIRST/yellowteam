package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;

@Config
public class Collection implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    private DcMotorEx collectorMotor;
    private ServoImplEx clutchLeft;
    private ServoImplEx clutchRight;

    //Swyft Sensors
    private DigitalChannel frontRightLaser;
    private DigitalChannel frontLeftLaser;
    private DigitalChannel backRightLaser;
    private DigitalChannel backLeftLaser;

    public CollectionState collectionState;
    public ClutchState clutchState;

    private double timerStart = 0;
    private boolean timerRunning = false;

    public static class Params{
        public double ENGAGED_POS = 0.1;
        public double DISENGAGED_POS = 0.9;
        public double DELAY_PERIOD = 0.5;
        public double INTAKE_SPEED = 0.25;
    }

    public static Params COLLECTOR_PARAMS = new Collection.Params();

    public Collection(HardwareMap hardwareMap, Telemetry telemetry){
        this.map = hardwareMap;
        this.telemetry = telemetry;

        collectorMotor = map.get(DcMotorEx.class, "intake");
        collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clutchRight = map.get(ServoImplEx.class, "clutchRight");
        clutchRight.setPwmRange(new PwmControl.PwmRange(1450, 1800));

        clutchLeft = map.get(ServoImplEx.class, "clutchLeft");
        clutchLeft.setPwmRange(new PwmControl.PwmRange(1450, 1800));

        frontRightLaser = hardwareMap.get(DigitalChannel.class, "FRLaser");
        frontRightLaser.setMode(DigitalChannel.Mode.INPUT);

        frontLeftLaser = hardwareMap.get(DigitalChannel.class, "FLLaser");
        frontLeftLaser.setMode(DigitalChannel.Mode.INPUT);

        backRightLaser = hardwareMap.get(DigitalChannel.class, "BRLaser");
        backRightLaser.setMode(DigitalChannel.Mode.INPUT);

        backLeftLaser = hardwareMap.get(DigitalChannel.class, "BLLaser");
        backLeftLaser.setMode(DigitalChannel.Mode.INPUT);

        collectionState = CollectionState.OFF;
        clutchState = ClutchState.UNENGAGED;
    }

    private boolean isBackBallDetected() {
        return !backLeftLaser.getState() || !backRightLaser.getState();
    }

    private boolean isFrontBallDetected() {
        return !frontLeftLaser.getState() || !frontRightLaser.getState();
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

    public void updateIntakeSequence(double currentTime) {
        if (collectionState == CollectionState.INTAKE) {
            if (isBackBallDetected() && isFrontBallDetected()) {
                if (!timerRunning) {
                    timerStart = currentTime;
                    timerRunning = true;
                } else if (currentTime - timerStart > COLLECTOR_PARAMS.DELAY_PERIOD) {
                    stopIntake();
                }
            } else {
                timerRunning = false;
                timerStart = 0;
            }
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
