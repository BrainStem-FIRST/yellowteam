package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Alliance;

@TeleOp(name="RED TELE", group="Competition")
public class RedTeleOp extends BrainSTEMTeleOp {

    public RedTeleOp() {
        super(Alliance.RED);
    }
}
