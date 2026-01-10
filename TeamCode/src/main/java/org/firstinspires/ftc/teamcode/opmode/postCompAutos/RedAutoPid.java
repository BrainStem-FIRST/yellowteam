package org.firstinspires.ftc.teamcode.opmode.postCompAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.Alliance;

@Autonomous(name="RED AUTO PID")
public class RedAutoPid extends AutoPid {
    @Override
    public void runOpMode() throws InterruptedException {
        alliance = Alliance.RED;
        super.runOpMode();
    }
}
