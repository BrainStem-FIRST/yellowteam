package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.Alliance;

@Autonomous(name="Blue Close Partner Auto")
public class BlueClosePartnerAuto extends ClosePartnerAuto {
    public BlueClosePartnerAuto() {
        super(Alliance.BLUE);
    }
}