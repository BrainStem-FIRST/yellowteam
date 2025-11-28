package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.Alliance;

@Autonomous(name="New Close RED Auto")
public class RedClosePartnerAuto extends ClosePartnerAuto {
    public RedClosePartnerAuto() {
        super(Alliance.RED);
    }
}