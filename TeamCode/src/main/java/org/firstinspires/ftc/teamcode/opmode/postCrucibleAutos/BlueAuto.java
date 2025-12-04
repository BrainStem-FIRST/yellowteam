package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.Alliance;

@Autonomous(name="BLUE auto")
public class BlueAuto extends AUTO {
    @Override
    public void runOpMode() throws InterruptedException {
        alliance = Alliance.BLUE;
        super.runOpMode();
    }
}
