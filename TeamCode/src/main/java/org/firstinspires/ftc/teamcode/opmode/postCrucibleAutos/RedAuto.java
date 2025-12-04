package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.Alliance;

@Autonomous(name="RED auto")
public class RedAuto extends AUTO {
    @Override
    public void runOpMode() throws InterruptedException {
        alliance = Alliance.RED;
        super.runOpMode();
    }
}