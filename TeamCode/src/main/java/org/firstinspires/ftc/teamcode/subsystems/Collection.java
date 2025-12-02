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

@Config
public class Collection extends Component {
    public static double shootOuttakeTime = 0.15;
    public static boolean activateLasers = true;
    private final DcMotorEx collectorMotor;
    private final ServoImplEx clutchLeft;
    private final ServoImplEx clutchRight;
    public ServoImplEx flickerRight;
    private final ServoImplEx flickerLeft;

    private final ElapsedTime flickerTimer = new ElapsedTime();
    private boolean flickerStarted = false;

    //Swyft Sensors (SET BOTH DIP SWITCHES TO 0)

    private final AnalogInput frontRightLaser;
    private final AnalogInput frontLeftLaser;
    private final AnalogInput backTopLaser;
    private final AnalogInput backBottomLaser;

    public CollectionState collectionState;
    public ClutchState clutchState;
    public FlickerState flickerState;
    public boolean extakeAfterClutchEngage;
    private double timerStart = 0;
    private boolean timerRunning = false;
    private boolean has3Balls = false;
    private final ElapsedTime timer = new ElapsedTime();
    public final ElapsedTime clutch_timer = new ElapsedTime();
    public double backLeftLaserDist, backRightLaserDist, frontLeftLaserDist, frontRightLaserDist;
    public static class Params{
        public double ENGAGED_POS = 0.1;
        public double DISENGAGED_POS = 0.95;
        public double DELAY_PERIOD = 0.2;
        public double INTAKE_SPEED = 0.95, FAR_ZONE_SHOOTER_ERROR_INTAKE_SPEED = 0;
        public double OUTTAKE_SPEED = -0.5;
        public double LASER_BALL_THRESHOLD = 2;
        public double flickerLeftMinPwm = 1643, flickerLeftMaxPwm = 1493;
        public double flickerRightMinPwm = 1491, flickerRightMaxPwm = 1641;
        public double flickerFullUpPos = 0.8;
        public double flickerHalfUpPos = 0.4;
        public double flickerDownPos = 0.05;
    }

    public static Params COLLECTOR_PARAMS = new Collection.Params();

    public Collection(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot){
        super(hardwareMap, telemetry, robot);

        collectorMotor = hardwareMap.get(DcMotorEx.class, "intake");
        collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clutchRight = hardwareMap.get(ServoImplEx.class, "clutchRight");
        clutchRight.setPwmRange(new PwmControl.PwmRange(1450, 2000));
        clutchLeft = hardwareMap.get(ServoImplEx.class, "clutchLeft");
        clutchLeft.setPwmRange(new PwmControl.PwmRange(1450, 2000));

        flickerRight = hardwareMap.get(ServoImplEx.class, "flickerRight");
        flickerRight.setPwmRange(new PwmControl.PwmRange(COLLECTOR_PARAMS.flickerRightMinPwm, COLLECTOR_PARAMS.flickerRightMaxPwm));
        flickerLeft = hardwareMap.get(ServoImplEx.class, "flickerLeft");
        flickerLeft.setPwmRange(new PwmControl.PwmRange(COLLECTOR_PARAMS.flickerLeftMinPwm, COLLECTOR_PARAMS.flickerLeftMaxPwm));

        frontRightLaser = hardwareMap.get(AnalogInput.class, "FRLaser");
        frontLeftLaser = hardwareMap.get(AnalogInput.class, "FLLaser");
        backTopLaser = hardwareMap.get(AnalogInput.class, "BRLaser");
        backBottomLaser = hardwareMap.get(AnalogInput.class, "BLLaser");

        collectionState = CollectionState.OFF;
        clutchState = ClutchState.UNENGAGED;
        flickerState = FlickerState.DOWN;

        timer.reset();
        clutch_timer.reset();
        clutchStateTimer = new ElapsedTime();
        clutchStateTimer.reset();
    }

    private double voltageToDistance(double voltage) {
        return (voltage * 43.92898) - 6.01454; //tune if not accurate
    }

    public boolean isBackBallDetected() {
        return backLeftLaserDist < COLLECTOR_PARAMS.LASER_BALL_THRESHOLD || backRightLaserDist < COLLECTOR_PARAMS.LASER_BALL_THRESHOLD;
    }

    private boolean isFrontBallDetected() {
        return frontRightLaserDist < COLLECTOR_PARAMS.LASER_BALL_THRESHOLD ||
                frontLeftLaserDist < COLLECTOR_PARAMS.LASER_BALL_THRESHOLD;
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
            } else if (currentTime - timerStart > COLLECTOR_PARAMS.DELAY_PERIOD)
                has3Balls = true;
        } else {
            timerRunning = false;
            timerStart = 0;
            has3Balls = false;
        }
    }

    public enum CollectionState {
        OFF, INTAKE, OUTTAKE, TRANSFER
    }

    public enum ClutchState {
        ENGAGED, UNENGAGED
    }

    public enum FlickerState {
        FULL_UP, DOWN, HALF_UP_DOWN, FULL_UP_DOWN
    }

    public final ElapsedTime clutchStateTimer;
    @Override
    public void printInfo() {}

    @Override
    public void reset() {
    }

    @Override
    public void update() {
        telemetry.addData("flicker state", flickerState);
        telemetry.addData("flicker left pos", flickerLeft.getPosition());
        telemetry.addData("flicker right pos", flickerRight.getPosition());

        backLeftLaserDist = voltageToDistance(backBottomLaser.getVoltage());
        backRightLaserDist = voltageToDistance(backTopLaser.getVoltage());
        frontLeftLaserDist = voltageToDistance(frontLeftLaser.getVoltage());
        frontRightLaserDist = voltageToDistance(frontRightLaser.getVoltage());

        switch (collectionState) {
            case OFF:
                collectorMotor.setPower(0);
                break;

            case INTAKE:
                double shooterError = Math.abs(robot.shooter.getAvgMotorVelocity() - robot.shooter.shooterPID.getTarget());
                double errorThreshold = robot.shooter.isNear ? Shooter.SHOOTER_PARAMS.maxErrorThresholdNear : Shooter.SHOOTER_PARAMS.maxErrorThresholdFar;
                telemetry.addData("shooter error", shooterError);
                if (clutchState == ClutchState.UNENGAGED || shooterError < errorThreshold)
                    collectorMotor.setPower(COLLECTOR_PARAMS.INTAKE_SPEED);
                else
                    collectorMotor.setPower(COLLECTOR_PARAMS.FAR_ZONE_SHOOTER_ERROR_INTAKE_SPEED);
                break;

            case OUTTAKE:
                collectorMotor.setPower(COLLECTOR_PARAMS.OUTTAKE_SPEED);
                break;

            case TRANSFER:
                collectorMotor.setPower(0.1);
                break;
        }

        switch (clutchState) {
            case ENGAGED:
                 if(clutch_timer.seconds() < shootOuttakeTime && extakeAfterClutchEngage)
                    collectionState = CollectionState.OUTTAKE;
                else if(collectionState == CollectionState.OUTTAKE)
                    collectionState = CollectionState.OFF;
                clutchRight.setPosition(COLLECTOR_PARAMS.ENGAGED_POS);
                clutchLeft.setPosition(COLLECTOR_PARAMS.ENGAGED_POS);
                break;

            case UNENGAGED:
                extakeAfterClutchEngage = true;
                clutch_timer.reset();
                clutchRight.setPosition(COLLECTOR_PARAMS.DISENGAGED_POS);
                clutchLeft.setPosition(COLLECTOR_PARAMS.DISENGAGED_POS);
                break;
        }

        switch (flickerState) {
            case FULL_UP:
                flickerLeft.setPosition(COLLECTOR_PARAMS.flickerFullUpPos);
                flickerRight.setPosition(COLLECTOR_PARAMS.flickerFullUpPos);
                break;
            case DOWN:
                flickerLeft.setPosition(COLLECTOR_PARAMS.flickerDownPos);
                flickerRight.setPosition(COLLECTOR_PARAMS.flickerDownPos);
                break;
            case HALF_UP_DOWN:
                if(!flickerStarted) {
                    flickerLeft.setPosition(COLLECTOR_PARAMS.flickerHalfUpPos);
                    flickerRight.setPosition(COLLECTOR_PARAMS.flickerHalfUpPos);
                    flickerTimer.reset();
                    flickerStarted = true;
                }
                else if(flickerTimer.seconds() > 0.3) {
                    flickerLeft.setPosition(COLLECTOR_PARAMS.flickerDownPos);
                    flickerRight.setPosition(COLLECTOR_PARAMS.flickerDownPos);
                    flickerStarted = false;
                    flickerState = FlickerState.DOWN;
                }
                break;
            case FULL_UP_DOWN:
                collectionState = CollectionState.OFF;
                if (!flickerStarted) {
                    flickerLeft.setPosition(COLLECTOR_PARAMS.flickerFullUpPos);
                    flickerRight.setPosition(COLLECTOR_PARAMS.flickerFullUpPos);
                    flickerTimer.reset();
                    flickerStarted = true;
                } else if (flickerTimer.seconds() > 0.4) {
                    flickerLeft.setPosition(COLLECTOR_PARAMS.flickerDownPos);
                    flickerRight.setPosition(COLLECTOR_PARAMS.flickerDownPos);
                    flickerStarted = false;
                    collectionState = CollectionState.INTAKE;
                    flickerState = FlickerState.DOWN;
                }
                break;
        }

        if (activateLasers)
            checkForIntakeBalls(timer.seconds());
//        // front 6.087 in
//
//        telemetry.addData("Back Ball Detected", isBackBallDetected());
//        telemetry.addData("Front Ball Detected", isFrontBallDetected());
//        telemetry.addData("FLICKER State", flickerState.toString());
//        telemetry.addData("Clutch State", clutchState.toString());
    }
}
