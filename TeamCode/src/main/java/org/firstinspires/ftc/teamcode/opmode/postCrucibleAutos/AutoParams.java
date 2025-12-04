package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

public class AutoParams {
    public static class Collect {
        public double firstNearMaxVel = 28, maxVel = 16;
        public double lineARed = Math.toRadians(90), lineABlue = 0;

        public double firstNearXRed = -14, postFirstNearYRed = 48.5, firstNearXBlue = -14, postFirstNearYBlue = -48;
        public double firstFarXRed = -15.5, preFirstFarYRed = 35, postFirstFarYRed = 54, firstFarXBlue = -16, preFirstFarYBlue = -47, postFirstFarYBlue = -54;

        public double preSecondYRed = 34, postSecondYRed = 52, preSecondYBlue = 0, postSecondYBlue = 0;
        public double secondNearXRed = 10, secondFarXRed = 12, secondNearXBlue = 12, secondFarXBlue = 12;
        public double secondBeginEndVel = 10;

        public double preThirdYRed = 32, preThirdYBlue = 0, postThirdYRed = 51, postThirdYBlue = 0;
        public double thirdNearXRed = 34, thirdFarXRed = 32, thirdNearXBlue = 34, thirdFarXBlue = 36;

        public double preLoadingXRed = 46, preLoadingYRed = 60, preLoadingARed = Math.toRadians(45), preLoadingXBlue = 50, loadingYBlue = 0, preLoadingABlue = 0;
        public double postLoadingXRed = 68, postLoadingYRed = 60, postLoadingARed = Math.toRadians(15), postLoadingXBlue = 0, postLoadingABlue = 0;

        public double gateCollectXRed = 68, gateCollectYRed = 65, gateCollectARed = Math.toRadians(90);
        public double gateCollectXBlue = 68, gateCollectYBlue = -65, gateCollectABlue = Math.toRadians(-90);
    }
    public static class Shoot {
        public double clutchDispPreloadNear = 25, clutchDisp1Near = 38, clutchDisp2Near = 41, clutchDisp3Near = 70, clutchDispLoadingNear = 105, clutchDispGateNear = 110;
        public double clutchDispPreloadFar = 1, clutchDisp1Far = 80, clutchDisp2Far = 67, clutchDisp3Far = 40, clutchDispLoadingFar = 48, clutchDispGateFar = 45;

        public double shootNearXRed = -15, shootNearYRed = 20, shootNearXBlue = 0, shootNearYBlue = 0;
        public double shootFarXRed = 55, shootFarYRed = 15, shootFarXBlue = 0, shootFarYBlue = 0;

        // custom shooting angles
        public double shootNearSetup1ARed = Math.toRadians(90), shootNearSetup2ARed = Math.toRadians(60), shootNearSetup3ARed = Math.toRadians(60), shootNearSetupLoadingARed = Math.toRadians(60);
        public double shootFarSetup1ARed = Math.toRadians(180), shootFarSetup2ARed = Math.toRadians(170), shootFarSetup3ARed = Math.toRadians(150), shootFarSetupLoadingARed = Math.toRadians(90);
        public double shootNearSetup1ABlue = Math.toRadians(160), shootNearSetup2ABlue = Math.toRadians(150), shootNearSetup3ABlue = Math.toRadians(135), shootNearSetupLoadingABlue = Math.toRadians(90);
        public double shootFarSetup1ABlue = Math.toRadians(-150), shootFarSetup2ABlue = Math.toRadians(-150), shootFarSetup3ABlue = Math.toRadians(-135), shootFarSetupLoadingABlue = Math.toRadians(-100);

        // custom shooting tolerances
        public double tolLoadingFarDist = 1.5, tolLoadingFarHeading = Math.toRadians(4);
    }

    public static class Misc {
        public double startNearXRed = -63.5, startNearYRed = 39.5, startNearARed = 0, startNearXBlue = 0, startNearYBlue = 0, startNearABlue = 0;
        public double startFarXRed = 62.1875, startFarYRed = 16.9859, startFarARed = Math.toRadians(180), startFarXBlue = 0, startFarYBlue = 0, startFarABlue = 0;
        public double gateNearX1Red = -4, gateNearX2Red = -1, gateNearYRed = 59,  gateNearX1Blue = 0, gateNearX2Blue = 0, gateNearYBlue = 0;
        public double gateFarX1Red = 0.5, gateFarX2Red = 3, gateFarYRed = 62, gateFarX1Blue = 0, gateFarX2Blue = 0, gateFarYBlue = 0;
        public double gateRedA1 = Math.toRadians(200), gateBlueA1 = Math.toRadians(-200), gateRedA2 = Math.toRadians(-20), gateBlueA2 = Math.toRadians(20);
        public double parkNearX = -12, parkFarX = 50;
        public double parkNearYRed = 36, parkNearARed = Math.toRadians(45), parkNearYBlue = -36, parkNearABlue = Math.toRadians(-45);
        public double parkFarYRed = 25, parkFarARed = Math.toRadians(135), parkFarYBlue = -25, parkFarABlue = Math.toRadians(-135);
    }
    public static class TimeConstraints {
        public double gateWait = 0;
        public double gateCollectMaxTime = 2.1;
        public double minShootTime = 0.5;
        public double ensureShootAll = 0.7;
        public double parkStartTime = 29.3;

    }
}
