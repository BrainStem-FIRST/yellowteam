package org.firstinspires.ftc.teamcode.utils.pidDrive;

import java.util.Arrays;

public class Tolerance {
    public static class DefaultParams {
        public double xTol = 1;
        public double yTol = 1;
        public double headingRadTol = Math.toRadians(2);
    }
    public static DefaultParams defaultParams = new DefaultParams();
    public double xTol;
    public double yTol;
    public double headingRadTol;
    public Tolerance(double xTol, double yTol, double headingRadTol) {
        this.xTol = xTol;
        this.yTol = yTol;
        this.headingRadTol = headingRadTol;
    }
    public Tolerance(double distTol, double headingRadTol) {
        this(distTol, distTol, headingRadTol);
    }
    public Tolerance(double[] tol) {
        if (tol.length == 1) {
            this.xTol = tol[0];
            this.yTol = tol[0];
            this.headingRadTol = defaultParams.headingRadTol;
        }
        else if (tol.length == 2) {
            this.xTol = tol[0];
            this.yTol = tol[0];
            this.headingRadTol = tol[1];
        }
        else if (tol.length == 3){
            this.xTol = tol[0];
            this.yTol = tol[1];
            this.headingRadTol = tol[2];
        }
        else
            throw new IllegalArgumentException("tolerance of " + Arrays.toString(tol) + " is not of right length (must contain 2 or 3 elements)");
    }

}
