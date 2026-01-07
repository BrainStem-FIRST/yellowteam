package org.firstinspires.ftc.teamcode.opmode.postCompAutos;

import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;

public class AutoParamsPid {
    public static class Collect {
        public double collectDrivePower = 0.5, waypointSlowDown = 0.3;
        public double collectWaypointTolX = 3.5, collectWaypointTolY = 3, collectWaypointTolHeading = 7;
        public Tolerance waypointTol = new Tolerance(collectWaypointTolX, collectWaypointTolY, collectWaypointTolHeading);
        public double lineARed = Math.toRadians(90), lineABlue = Math.toRadians(-90);
        public double firstXRed = -12, preFirstYRed = 31, postFirstYRed = 49;
        public double firstXBlue = -12, preFirstYBlue = -31, postFirstYBlue = -49;
        public double secondXRed = 12, preSecondYRed = 29, postSecondYRed = 48;
        public double secondXBlue = 12, preSecondYBlue = -29, postSecondYBlue = -48;
        public double thirdXRed = 36, preThirdYRed = 30, postThirdYRed = 52;
        public double thirdXBlue = 36, preThirdYBlue = -30, postThirdYBlue = -52;
        public double preCollect2NearXOffset = 2.5, preCollect3NearXOffset = 4;
        public double preCollect3NearARed = Math.toRadians(85);
        public double preCollect3NearABlue = Math.toRadians(-85);

        public double preLoadingXRed = 46.5, preLoadingYRed = 60, preLoadingARed = Math.toRadians(50), preLoadingXBlue = 46.5, preLoadingYBlue = -60, preLoadingABlue = Math.toRadians(-50);
        public double postLoadingXRed = 68, postLoadingYRed = 60, postLoadingARed = Math.toRadians(16), postLoadingXBlue = 68, postLoadingYBlue = -60, postLoadingABlue = Math.toRadians(-12);
        public double cornerCollectXRed = 66, cornerCollectYRed = 68, cornerCollectARed = Math.toRadians(90);
        public double cornerCollectXBlue = 66, cornerCollectYBlue = -68, cornerCollectABlue = Math.toRadians(-90);
        public double cornerCollectRetryX = 55, cornerCollectRetryYRed = 46, cornerCollectRetryYBlue = -46;

        public double gateCollectWaypointDistTol = 5, gateCollectWaypointHeadingTol = 20;
        public Tolerance gateCollectWaypointTol = new Tolerance(gateCollectWaypointDistTol, gateCollectWaypointHeadingTol);
        public double gateCollectWaypointXRed = -3, gateCollectWaypointYRed = 39;
        public double gateCollectWaypointXBlue = 0, gateCollectWaypointYBlue = -34;
        public double gateCollectOpenXRed = 11, gateCollectOpenYRed = 62, gateCollectOpenARed = Math.toRadians(114.5);
        public double gateCollectOpenXBlue = 11, gateCollectOpenYBlue = -62.5, gateCollectOpenABlue = Math.toRadians(-116);
        public double gateCollectXRed = 12, gateCollectYRed = 62, gateCollectARed = Math.toRadians(135);
        public double gateCollectXBlue = 12, gateCollectYBlue = -62, gateCollectABlue = Math.toRadians(-135);
        public double gateCollectBackupXRed = 8, gateCollectBackupYRed = -60;
        public double gateCollectBackupXBlue = 8, gateCollectBackupYBlue = 60;

    }
    public static class Shoot {
        public int bestPoseNumComputations = 50;
        public double waypointTolX = 3, waypointTolY = 3, waypointTolA = Math.toRadians(5);
        public Tolerance waypointTol = new Tolerance(waypointTolX, waypointTolY, waypointTolA);
        public double waypointSlowDown = 0.3;
        // shooting positions
        public double shootNearXRed = -15, shootNearYRed = 23, shootNearXBlue = -15, shootNearYBlue = -23;
        public double shootMidXRed = -8, shootMidYRed = 18.5, shootMidXBlue = -8, shootMidYBlue = -18.5;
        public double shootFarXRed = 54, shootFarYRed = 16, shootFarXBlue = 54, shootFarYBlue = -16;

        // custom shooting angles
        public double shootNearSetup1ARed = Math.toRadians(90), shootNearSetup2ARed = Math.toRadians(60), shootNearSetup3ARed = Math.toRadians(60), shootNearSetupLoadingARed = Math.toRadians(60);
        public double shootFarSetup1ARed = Math.toRadians(180), shootFarSetup2ARed = Math.toRadians(170), shootFarSetup3ARed = Math.toRadians(150), shootFarSetupLoadingARed = Math.toRadians(95);
        public double shootNearSetup1ABlue = Math.toRadians(-90), shootNearSetup2ABlue = Math.toRadians(-70), shootNearSetup3ABlue = Math.toRadians(-70), shootNearSetupLoadingABlue = Math.toRadians(-60);
        public double shootFarSetup1ABlue = Math.toRadians(-180), shootFarSetup2ABlue = Math.toRadians(-170), shootFarSetup3ABlue = Math.toRadians(-150), shootFarSetupLoadingABlue = Math.toRadians(-95);

        // shooting path waypoints to not hit other balls
        public double shootFar1WaypointXRed = 0, shootFar1WaypointYRed = 24, shootFar1WaypointARed = Math.toRadians(140);
        public double shootFar2WaypointXRed = 24, shootFar2WaypointYRed = 24, shootFar2WaypointARed = Math.toRadians(150);
        public double shoot2NearWaypointXRed = -5, shoot2NearWaypointYRed = 40, shoot2NearWaypointARed = Math.toRadians(60);

        public double shootFar1WaypointXBlue = 0, shootFar1WaypointYBlue = -24, shootFar1WaypointABlue = Math.toRadians(-140);
        public double shootFar2WaypointXBlue = 24, shootFar2WaypointYBlue = -24, shootFar2WaypointABlue = Math.toRadians(-150);
        public double shoot2NearWaypointXBlue = -5, shoot2NearWaypointYBlue = -40, shoot2NearWaypointABlue = Math.toRadians(-60);
    }

    public static class Misc {
        public double startNearXRed = -61.5, startNearYRed = 41.1, startNearARed = 0, startNearXBlue = -61.5, startNearYBlue = -41.1, startNearABlue = 0;
        public double startFarXRed = 60.8, startFarYRed = 15.4, startFarARed = Math.toRadians(180), startFarXBlue = 60.85, startFarYBlue = -19.2, startFarABlue = Math.toRadians(-180);
        public double gate1XRed = -3, gate2XRed = 5, gateYRed = 57;
        public double gate1XBlue = -3, gate2XBlue = 5, gateYBlue = -57;
        public double gateARed = Math.toRadians(90), gateABlue = Math.toRadians(-90);
        public double preGateClearance = 5, preGateXOffset = -5;
        public double preGateTolX = 3.5, preGateTolY = 3.5, preGateTolHeading = 7;
        public Tolerance preGateTol = new Tolerance(preGateTolX, preGateTolY, preGateTolHeading);
        public double gateCollectWaypointMinPower = 0.7, gateMinPower = 0.5;
        public double parkNearXRed = -22, parkNearYRed = 36, parkNearARed = Math.toRadians(45);
        public double parkNearXBlue = -22, parkNearYBlue = -36, parkNearABlue = Math.toRadians(-45);
        public double parkFarXRed = 50, parkFarYRed = 30, parkFarARed = Math.toRadians(135);
        public double parkFarXBlue = 50, parkFarYBlue = -30, parkFarABlue = Math.toRadians(-135);
    }
    public static class TimeConstraints {
        public double gateOpeningWait = 0.5;
        public double gateCollectMaxTime = 3;
        public double cornerCollectMaxTime = 1.9;
        public double minShootTime = 0.5;
        public double parkStartTime = 29.5;
        public double stopEverythingTime = 35;
        public double postIntakeTime = 0.4, loadingSlowIntakeTime = 1;

    }
}

