package org.firstinspires.ftc.teamcode.utils.math;

public class TwoPointLine {
    public final double x1, y1, x2, y2;
    public final double slope, yInt;
    public TwoPointLine(double x1, double y1, double x2, double y2) {
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;

        slope = (y2 - y1) / (x2 - x1);
        yInt = y1 - slope * x1;
    }
    public double calculate(double x) {
        return slope * x + yInt;
    }
}
