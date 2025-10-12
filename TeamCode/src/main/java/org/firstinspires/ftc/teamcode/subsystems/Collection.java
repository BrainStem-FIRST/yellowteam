package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;

@Config
public class Collection implements Component {

    private HardwareMap map;
    private Telemetry telemetry;
    private DcMotorEx collectorMotor;
    private ServoImplEx shifter;

    //Swyft Sensors
    private DigitalChannel frontRightLaser;
    private DigitalChannel frontLeftLaser;
    private DigitalChannel backRightLaser;
    private DigitalChannel backLeftLaser;

    public CollectionState collectionState;

    private double timerStart = 0;
    private boolean timerRunning = false;

    public static class Params{
        private double LOCK_POSITION = 0;
        private double UNLOCK_POSITION = 1;
        private double DELAY_PERIOD = 0.5;
    }

    public static Params COLLECTOR_PARAMS = new Collection.Params();

    public Collection(HardwareMap hardwareMap, Telemetry telemetry){
        this.map = hardwareMap;
        this.telemetry = telemetry;

        collectorMotor = map.get(DcMotorEx.class, "collection");
        collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shifter = map.get(ServoImplEx.class, "shifter");

        frontRightLaser = hardwareMap.get(DigitalChannel.class, "frontRightLaser");
        frontRightLaser.setMode(DigitalChannel.Mode.INPUT);

        frontLeftLaser = hardwareMap.get(DigitalChannel.class, "frontLeftLaser");
        frontLeftLaser.setMode(DigitalChannel.Mode.INPUT);

        backRightLaser = hardwareMap.get(DigitalChannel.class, "backRightLaser");
        backRightLaser.setMode(DigitalChannel.Mode.INPUT);

        backLeftLaser = hardwareMap.get(DigitalChannel.class, "backLeftLaser");
        backLeftLaser.setMode(DigitalChannel.Mode.INPUT);
    }

    private boolean isBackBallDetected() {
        return !backLeftLaser.getState() || !backRightLaser.getState();
    }

    private boolean isFrontBallDetected() {
        return !frontLeftLaser.getState() || !frontRightLaser.getState();
    }

    public boolean isCollectorFull(double confirmationDelay, double startTime, double currentTime) {
        if (isBackBallDetected() && isFrontBallDetected()) {
            return (currentTime - startTime) > confirmationDelay;
        }
        return false;
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

    @Override
    public void reset() {
    }

    @Override
    public void update() {

        switch (collectionState) {
            case OFF: {
                collectorMotor.setPower(0);
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
                break;
            }
        }

        telemetry.addData("Back Ball Detected", isBackBallDetected());
        telemetry.addData("Front Ball Detected", isFrontBallDetected());
        telemetry.update();
    }

    @Override
    public String test(){
        return null;
    }
}
