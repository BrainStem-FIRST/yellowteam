package org.firstinspires.ftc.teamcode.utils.misc;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoImplCacher {
    private double pos;
    private final ServoImplEx servo;
    public ServoImplCacher(ServoImplEx servo) {
        this.servo = servo;
    }
    public void updateProperties() {
        pos = servo.getPosition();
    }
    public double getPosition() {
        return pos;
    }
    public void setPosition(double p) {
        servo.setPosition(p);
    }
}
