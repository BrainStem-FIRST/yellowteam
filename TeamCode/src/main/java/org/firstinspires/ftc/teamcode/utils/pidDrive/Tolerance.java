package org.firstinspires.ftc.teamcode.utils.pidDrive;

public class Tolerance {
    public static class DefaultParams {
        public double xTol = 1;
        public double yTol = 1;
        public double headingDegTol = 3;
    }
    public static DefaultParams defaultParams = new DefaultParams();
    public double xTol;
    public double yTol;
    public double headingDegTol;
    public Tolerance(double xTol, double yTol, double headingDegTol) {
        this.xTol = xTol;
        this.yTol = yTol;
        this.headingDegTol = headingDegTol;
    }
    public Tolerance(double distTol, double headingDegTol) {
        this(distTol, distTol, headingDegTol);
    }
    public Tolerance(double distTol) {
        this(distTol, distTol, defaultParams.headingDegTol);
    }
}
