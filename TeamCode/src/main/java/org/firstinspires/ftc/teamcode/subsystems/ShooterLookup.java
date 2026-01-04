package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;

@Config
public class ShooterLookup {
    public static double[] dists =    {40,   45,   50,   54,   62,   68,   74.4, 81,   87,   99};
    public static double[] vels =     {1250, 1280, 1280, 1310, 1370, 1430, 1490, 1530, 1580, 1650};
    public static double[] exitAngs = {1.1,  1,    0.1,  0.98, 0.95, 0.93, 0.87, 0.81, 0.75, 0.7};
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
