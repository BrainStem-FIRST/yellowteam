package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.teleop.BrainSTEMTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightLocalization;

@Config
public class LED extends Component {
    public static double white = 0.99, green = 0.45, yellow = 0.35, blue = 0.6, purple = 0.666, red = 0.279;
    public static double shooterFlashOnTime = 0.3, shooterFlashOffTime = 0.1;
    public static double turretFlashOnTime = 0.1, turretFlashOffTime = 0.1;
    public static double confirmSuccessfulPoseUpdateTime = 0.2;
    private final ServoImplEx left_led;
    private final ServoImplEx right_led;
    private final ElapsedTime shooterFlashTimer, turretFlashTimer;
    public double lastPinpointResetTimeMs;

    public LED(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);

        right_led = hardwareMap.get(ServoImplEx.class, "rightLED");
        left_led = hardwareMap.get(ServoImplEx.class, "leftLED");
        shooterFlashTimer = new ElapsedTime();
        shooterFlashTimer.reset();
        turretFlashTimer = new ElapsedTime();
        turretFlashTimer.reset();
        lastPinpointResetTimeMs = -1000000;
    }

    @Override
    public void printInfo() {}

    public void update(boolean limelightUpdatingPose, boolean confirmLimelightPoseUpdate, boolean shooterErrorInRange, boolean turretInRange, boolean clutchEngaged, boolean intaking, boolean has3Balls) {
        //robot.limelight.localization.getState() == LimelightLocalization.LocalizationState.UPDATING_POSE
        //robot.limelight.localization.getPrevState() == LimelightLocalization.LocalizationState.UPDATING_POSE &&
        //                robot.limelight.localization.successfullyFoundPose &&
        //                robot.limelight.localization.getStateTime() < confirmSuccessfulPoseUpdateTime
        // robot.shooter.shooterState == Shooter.ShooterState.UPDATE && error > BrainSTEMTeleOp.firstShootTolerance
        if (limelightUpdatingPose) {
            setLed(white);
            return;
        }
        if (confirmLimelightPoseUpdate) {
            setLed(blue);
            return;
        }
        if (System.currentTimeMillis() - lastPinpointResetTimeMs < 200) {
            setLed(blue);
            return;
        }
        if (!shooterErrorInRange) {
            if (shooterFlashTimer.seconds() > shooterFlashOnTime + shooterFlashOffTime)
                shooterFlashTimer.reset();
            else if (shooterFlashTimer.seconds() > shooterFlashOnTime) {
                setLed(0);
                return;
            }
        }
        if(!turretInRange) {
            if(turretFlashTimer.seconds() > turretFlashOnTime + turretFlashOffTime)
                turretFlashTimer.reset();
            else if(turretFlashTimer.seconds() > turretFlashOnTime) {
                setLed(0);
                return;
            }
        }
        if (clutchEngaged) {
            if (intaking)
                setLed(green);
            else
                setLed(yellow);
        }
        else if (has3Balls)
                setLed(purple);
            else
                setLed(red);
    }
    public void setLed(double position) {
        left_led.setPosition(position);
        right_led.setPosition(position);
    }
}
