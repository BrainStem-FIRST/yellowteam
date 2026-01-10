package org.firstinspires.ftc.teamcode.opmode.postCompAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.Alliance;

@Autonomous(name="BLUE AUTO PID")
public class BlueAutoPid extends AutoPid {

    @Override
    public void runOpMode() throws InterruptedException {
        alliance = Alliance.BLUE;
        super.runOpMode();
    }
}
