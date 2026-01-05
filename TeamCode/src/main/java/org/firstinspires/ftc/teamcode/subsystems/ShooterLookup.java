package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;

@Config
public class ShooterLookup {
    public static double[] dists =    {29.6, 34,   39,   44,   49,   53,   61,   67,   73.4, 80,   86,   95.69,98,   104,  109,  128,  136,  138,  146.5, 155  };
    public static double[] vels =     {1250, 1260, 1250, 1280, 1280, 1310, 1370, 1430, 1490, 1530, 1580, 1636, 1650, 1700, 1720, 1860, 1910, 1900, 1960, 2010 };
    public static double[] exitAngs = {1.25, 1.15, 1.1,  1,    1,    0.98, 0.95, 0.93, 0.87, 0.81, 0.75, 0.724,0.7,  0.7,  0.7,  0.64, 0.64, 0.64, 0.64, 0.64 };
    private final InterpLUT velocityLookup;
    private final InterpLUT exitAngleLookup;
    public ShooterLookup() {
        velocityLookup = new InterpLUT();
        exitAngleLookup = new InterpLUT();

        for(int i = 0; i < dists.length; i++) {
            velocityLookup.add(dists[i], vels[i]);
            exitAngleLookup.add(dists[i], exitAngs[i]);
        }
        velocityLookup.createLUT();
        exitAngleLookup.createLUT();
    }

    public double lookupVelocityTicksPerSec(double dist) {
        return velocityLookup.get(dist);
    }
    public double lookupExitAngleRad(double dist) {
        return exitAngleLookup.get(dist);
    }

}
