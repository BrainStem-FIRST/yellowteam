package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.Alliance;

@Autonomous(name="New Close RED Auto")
public class RedClosePartnerAuto extends CloseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        alliance = Alliance.RED;
        super.runOpMode();
    }
}