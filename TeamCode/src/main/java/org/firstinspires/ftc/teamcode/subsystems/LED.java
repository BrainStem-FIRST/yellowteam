package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.teleop.BrainSTEMTeleOp;

@Config
public class LED extends Component {
    public static double white = 0.99, green = 0.45, yellow = 0.35, blue = 0.6, purple = 0.666, red = 0.279, orange = 0.16;
    public static double flashOnTime = 0.3, flashOffTime = 0.1;
    public static double confirmSuccessfulPoseUpdateTime = 0.2;
    private final ServoImplEx left_led;
    private final ServoImplEx right_led;
    private final ElapsedTime timer;
    public LED(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);

        right_led = hardwareMap.get(ServoImplEx.class, "rightLED");
        left_led = hardwareMap.get(ServoImplEx.class, "leftLED");
        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void printInfo() {}

    @Override
    public void update(){
        if (robot.limelight.getState() == Limelight.UpdateState.UPDATING_POSE) {
            setLed(white);
            return;
        }
        if (robot.limelight.getPrevState() == Limelight.UpdateState.UPDATING_POSE &&
                robot.limelight.successfullyFoundPose &&
                robot.limelight.getStateTime() < confirmSuccessfulPoseUpdateTime) {
            setLed(blue);
            return;
        }

        double error = Math.abs(robot.shooter.shooterPID.getTarget() - robot.shooter.getAvgMotorVelocity());
        if (robot.shooter.shooterState == Shooter.ShooterState.UPDATE && error > BrainSTEMTeleOp.firstShootTolerance) {
            if (timer.seconds() > flashOnTime + flashOffTime)
                timer.reset();
            else if (timer.seconds() > flashOnTime) {
                setLed(0);
                return;
            }
        }
        if (robot.collection.clutchState == Collection.ClutchState.ENGAGED) {
            if (robot.collection.collectionState == Collection.CollectionState.INTAKE)
                setLed(green);
            else
                setLed(yellow);
        }
        else if (robot.collection.intakeHas3Balls())
                setLed(purple);
            else
                setLed(red);
    }
    public void setLed(double position) {
        left_led.setPosition(position);
        right_led.setPosition(position);
    }
}
