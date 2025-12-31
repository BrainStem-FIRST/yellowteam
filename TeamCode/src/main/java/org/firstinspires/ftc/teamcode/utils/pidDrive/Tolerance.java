package org.firstinspires.ftc.teamcode.utils.pidDrive;

import com.acmerobotics.dashboard.config.Config;

import java.util.Arrays;

@Config
public class Tolerance {
    public static class DefaultParams {
        public double xTol = 2;
        public double yTol = 2;
        public double headingDegTol = 5;
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
    public Tolerance(double[] tol) {
        if (tol.length == 1) {
            this.xTol = tol[0];
            this.yTol = tol[0];
            this.headingDegTol = defaultParams.headingDegTol;
        }
        else if (tol.length == 2) {
            this.xTol = tol[0];
            this.yTol = tol[0];
            this.headingDegTol = tol[1];
        }
        else if (tol.length == 3){
            this.xTol = tol[0];
            this.yTol = tol[1];
            this.headingDegTol = tol[2];
        }
        else
            throw new IllegalArgumentException("tolerance of " + Arrays.toString(tol) + " is not of right length (must contain 2 or 3 elements)");
    }

}
