package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

import org.firstinspires.ftc.teamcode.opmode.Alliance;

public class FarAutoBlue extends FarAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        alliance = Alliance.BLUE;
        super.runOpMode();
    }
}
