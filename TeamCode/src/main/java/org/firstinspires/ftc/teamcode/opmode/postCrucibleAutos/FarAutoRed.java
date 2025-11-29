package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

import org.firstinspires.ftc.teamcode.opmode.Alliance;

public class FarAutoRed extends FarAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        alliance = Alliance.RED;
        super.runOpMode();
    }
}
