package org.firstinspires.ftc.teamcode.utils.math;

public class HeadingCorrect {
    /**
     * ensures the angle is between [0, 2PI)
     **/
    public static double normalizeHeading(double rawHeading) {
        int numTimesOver = (int) (rawHeading / (Math.PI * 2));
        double newRad = rawHeading - Math.PI * 2 * numTimesOver;
        if (newRad < 0)
            newRad += Math.PI * 2;
        return newRad;
    }

    /**
     * <p>
     * example A: if rawErrorRad = rad(350) - rad(10) = rad(340), newRad = (rad(360) - rad(340)) * -1 = -rad(20)
     * NOTICE: the sign of the ORIGINAL error is positive, but the sign of the CORRECTED error is negative
     * </p>
     * <p>
     * example B: if rawErrorRad = rad(180) - rad(140) = rad(40), newRad = rad(40)
     * this is a normal angle, so no correction needs to be done
     * </p>
     * @param rawErrorRad target heading rad - current heading rad OR (when correcting heading velocity): current heading rad - previous heading rad
     * @return corrected directional heading error
     **/
    public static double correctHeadingErrorRad(double rawErrorRad) {
        double newRad = rawErrorRad;
        // then
        if (Math.abs(rawErrorRad) > Math.PI) {
            newRad = Math.PI * 2 - rawErrorRad;
            newRad *= -1;
        }
        return newRad;
    }
}
