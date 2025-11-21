package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.Collection;
import org.firstinspires.ftc.teamcode.utils.teleHelpers.GamepadTracker;

@Config
@TeleOp(name="raw subsystem test")
public class RawSubsystemTest extends LinearOpMode {
    public static double intakePower = 0.99;
    public static double shooterPower = 0.9;
    public static int shooterDirFlip = -1;
    public static double hoodPos = 0.93;
    public static int hoodLower = 1000, hoodUpper = 2200;
    public static double hoodInc = 0.01;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "lowShoot");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "highShoot");

        ServoImplEx hoodLeft = hardwareMap.get(ServoImplEx.class, "hoodLeft");
        hoodLeft.setPwmRange(new PwmControl.PwmRange(hoodLower, hoodUpper));
        ServoImplEx hoodRight = hardwareMap.get(ServoImplEx.class, "hoodRight");
        hoodRight.setPwmRange(new PwmControl.PwmRange(hoodLower, hoodUpper));

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        DcMotorEx collectorMotor = hardwareMap.get(DcMotorEx.class, "intake");
        collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ServoImplEx clutchRight = hardwareMap.get(ServoImplEx.class, "clutchRight");
        clutchRight.setPwmRange(new PwmControl.PwmRange(1450, 2000));

        ServoImplEx clutchLeft = hardwareMap.get(ServoImplEx.class, "clutchLeft");
        clutchLeft.setPwmRange(new PwmControl.PwmRange(1450, 2000));

        GamepadTracker g1 = new GamepadTracker(gamepad1);

        waitForStart();
        while(opModeIsActive()) {
            g1.update();

            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower * shooterDirFlip);

            if(g1.isFirstDpadUp())
                hoodPos += hoodInc;
            else if(g1.isFirstDpadDown())
                hoodPos -= hoodInc;

            if(g1.isFirstA())
                if(Math.abs(collectorMotor.getPower()) > 0.5)
                    collectorMotor.setPower(0);
                else
                    collectorMotor.setPower(intakePower);

            else if(g1.isFirstB())
                if(clutchLeft.getPosition() > 0.5) {
                    clutchLeft.setPosition(Collection.COLLECTOR_PARAMS.ENGAGED_POS);
                    clutchRight.setPosition(Collection.COLLECTOR_PARAMS.ENGAGED_POS);
                }
                else {
                    clutchLeft.setPosition(Collection.COLLECTOR_PARAMS.DISENGAGED_POS);
                    clutchRight.setPosition(Collection.COLLECTOR_PARAMS.DISENGAGED_POS);

                }
            hoodLeft.setPosition(hoodPos);
            hoodRight.setPosition(hoodPos);
            telemetry.addData("1 power", shooter1.getPower());
            telemetry.addData("2 power", shooter2.getPower());
            telemetry.addData("1 vel", shooter1.getVelocity());
            telemetry.addData("2 vel", shooter2.getVelocity());
            telemetry.addData("1 encoder", shooter1.getCurrentPosition());
            telemetry.addData("2 encoder", shooter2.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("hoodL pos", hoodLeft.getPosition());
            telemetry.addData("hoodR pos", hoodRight.getPosition());
            telemetry.addLine();
            telemetry.addData("intake power", collectorMotor.getPower());
            telemetry.addData("left clutch pos", clutchLeft.getPosition());
            telemetry.addData("right clutch pos", clutchRight.getPosition());
            telemetry.update();
        }
    }
}
