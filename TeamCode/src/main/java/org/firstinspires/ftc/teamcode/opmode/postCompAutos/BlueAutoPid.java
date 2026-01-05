package org.firstinspires.ftc.teamcode.opmode.postCompAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmode.Alliance;

@Autonomous(name="blue auto pid")
public class BlueAutoPid extends AutoPid {

    @Override
    public void runOpMode() throws InterruptedException {
        alliance = Alliance.BLUE;
        super.runOpMode();
    }
}
