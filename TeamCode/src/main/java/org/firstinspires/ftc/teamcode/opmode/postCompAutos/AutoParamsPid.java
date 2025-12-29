package org.firstinspires.ftc.teamcode.opmode.postCompAutos;

import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;

public class AutoParamsPid {
    public static class Collect {
        public double collectMaxPower = 0.5, waypointSlowDown = 0.3;
        public double collectWaypointTolX = 3, collectWaypointTolY = 3, collectWaypointTolHeading = Math.toRadians(5);
        public Tolerance waypointTol = new Tolerance(collectWaypointTolX, collectWaypointTolY, collectWaypointTolHeading);
        public double lineARed = Math.toRadians(90), lineABlue = Math.toRadians(-90);
        public double firstXRed = -12, preFirstYRed = 32, postFirstYRed = 48.5;
        public double firstXBlue = -12, preFirstYBlue = -38, postFirstYBlue = -48.5;
        public double secondXRed = 12, preSecondYRed = 32, postSecondYRed = 52;
        public double secondXBlue = 12, preSecondYBlue = -32, postSecondYBlue = -52;
        public double thirdXRed = 36, preThirdYRed = 32, postThirdYRed = 52;
        public double thirdXBlue = 36, preThirdYBlue = -32, postThirdYBlue = -52;

        public double preLoadingXRed = 46.5, preLoadingYRed = 60, preLoadingARed = Math.toRadians(50), preLoadingXBlue = 46.5, preLoadingYBlue = -60, preLoadingABlue = Math.toRadians(-50);
        public double postLoadingXRed = 68, postLoadingYRed = 60, postLoadingARed = Math.toRadians(16), postLoadingXBlue = 68, postLoadingYBlue = -60, postLoadingABlue = Math.toRadians(-12);


        public double gateCollectXRed = 66, gateCollectYRed = 68, gateCollectARed = Math.toRadians(90);
        public double gateCollectXBlue = 66, gateCollectYBlue = -68, gateCollectABlue = Math.toRadians(-90);
        public double gateCollectRetryX = 60, gateCollectRetryYRed = 46, gateCollectRetryYBlue = -46;
    }
    public static class Shoot {
        public double waypointTolX = 3, waypointTolY = 3, waypointTolA = Math.toRadians(5);
        public Tolerance waypointTol = new Tolerance(waypointTolX, waypointTolY, waypointTolA);
        public double waypointSlowDown = 0.3;
        // shooting positions
        public double shootNearXRed = -15, shootNearYRed = 20, shootNearXBlue = -15, shootNearYBlue = -20;
        public double shootFarXRed = 54, shootFarYRed = 16, shootFarXBlue = 54, shootFarYBlue = -16;

        // custom shooting angles
        public double shootNearSetup1ARed = Math.toRadians(90), shootNearSetup2ARed = Math.toRadians(60), shootNearSetup3ARed = Math.toRadians(60), shootNearSetupLoadingARed = Math.toRadians(60);
        public double shootFarSetup1ARed = Math.toRadians(180), shootFarSetup2ARed = Math.toRadians(170), shootFarSetup3ARed = Math.toRadians(150), shootFarSetupLoadingARed = Math.toRadians(95);
        public double shootNearSetup1ABlue = Math.toRadians(-90), shootNearSetup2ABlue = Math.toRadians(-60), shootNearSetup3ABlue = Math.toRadians(-60), shootNearSetupLoadingABlue = Math.toRadians(-60);
        public double shootFarSetup1ABlue = Math.toRadians(-180), shootFarSetup2ABlue = Math.toRadians(-170), shootFarSetup3ABlue = Math.toRadians(-150), shootFarSetupLoadingABlue = Math.toRadians(-95);

        // shooting path waypoints to not hit other balls
        public double shootFar1WaypointXRed = 0, shootFar1WaypointYRed = 24, shootFar1WaypointARed = Math.toRadians(140);
        public double shootFar2WaypointXRed = 24, shootFar2WaypointYRed = 24, shootFar2WaypointARed = Math.toRadians(150);

        public double shootFar1WaypointXBlue = 0, shootFar1WaypointYBlue = -24, shootFar1WaypointABlue = Math.toRadians(-140);
        public double shootFar2WaypointXBlue = 24, shootFar2WaypointYBlue = -24, shootFar2WaypointABlue = Math.toRadians(-150);
    }

    public static class Misc {
        public double startNearXRed = -63.5, startNearYRed = 39.5, startNearARed = 0, startNearXBlue = -63.5, startNearYBlue = -40, startNearABlue = 0;
        public double startFarXRed = 60.8, startFarYRed = 15.4, startFarARed = Math.toRadians(180), startFarXBlue = 60.85, startFarYBlue = -19.2, startFarABlue = Math.toRadians(-180);
        public double gate1XRed = -2.5, gate2XRed = 2.5, gateYRed = 53;
        public double gate1XBlue = -2.5, gate2XBlue = 2.5, gateYBlue = -53;
        public double gate1A = Math.toRadians(0), gate2A = Math.toRadians(180);
        public double parkNearXRed = -18, parkNearYRed = 36, parkNearARed = Math.toRadians(45);
        public double parkNearXBlue = -18, parkNearYBlue = -36, parkNearABlue = Math.toRadians(-45);
        public double parkFarXRed = 50, parkFarYRed = 30, parkFarARed = Math.toRadians(135);
        public double parkFarXBlue = 50, parkFarYBlue = -30, parkFarABlue = Math.toRadians(-135);
    }
    public static class TimeConstraints {
        public double gateWait = 0;
        public double gateCollectMaxTime = 1.9;
        public double minShootTime = 0.5;
        public double parkStartTime = 29.5;
        public double stopEverythingTime = 35;
        public double slowIntakeTime = 0.15, loadingSlowIntakeTime = 1;

    }
}

