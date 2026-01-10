package org.firstinspires.ftc.teamcode.opmode.postCompAutos;

import org.firstinspires.ftc.teamcode.utils.pidDrive.PathParams;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;

public class AutoParamsPid {
    public static class Collect {
        public double collectDrivePower = 0.53, waypointSlowDown = 0.4;
        public double collectWaypointTolX = 1, collectWaypointTolY = 2, collectWaypointTolHeading = 7;
        public Tolerance waypointTol = new Tolerance(collectWaypointTolX, collectWaypointTolY, collectWaypointTolHeading);
        public double lineARed = Math.toRadians(90), lineABlue = Math.toRadians(-90);
        public double firstXRed = -12, preFirstYRed = 31, postFirstYRed = 49;
        public double firstXBlue = -12, preFirstYBlue = -31, postFirstYBlue = -49;
        public double secondXRed = 12, preSecondYRed = 27.5, postSecondYRed = 48;
        public double secondXBlue = 12, preSecondYBlue = -28, postSecondYBlue = -48;
        public double thirdXRed = 36, preThirdYRed = 28, postThirdYRed = 52;
        public double thirdXBlue = 36, preThirdYBlue = -30, postThirdYBlue = -52;
        public double preCollect2NearRedXOffset = 4, preCollect2NearBlueXOffset = 2.5, preCollect3NearXOffset = 4;
        public double preCollect3NearARed = Math.toRadians(85);
        public double preCollect3NearABlue = Math.toRadians(-85);

        public double preLoadingXRed = 46.5, preLoadingYRed = 60, preLoadingARed = Math.toRadians(50), preLoadingXBlue = 46.5, preLoadingYBlue = -60, preLoadingABlue = Math.toRadians(-50);
        public double postLoadingXRed = 68, postLoadingYRed = 60, postLoadingARed = Math.toRadians(16), postLoadingXBlue = 68, postLoadingYBlue = -60, postLoadingABlue = Math.toRadians(-12);
        public double cornerCollectXRed = 66, cornerCollectYRed = 68, cornerCollectARed = Math.toRadians(90);
        public double cornerCollectXBlue = 66, cornerCollectYBlue = -68, cornerCollectABlue = Math.toRadians(-90);
        public double cornerCollectRetryX = 55, cornerCollectRetryYRed = 46, cornerCollectRetryYBlue = -46;
        public double gateCollectDistTol = 1, gateCollectHeadingTol = 2;
        public Tolerance gateCollectTol = new Tolerance(gateCollectDistTol, gateCollectHeadingTol);
        public double gateCollectWaypointXRed = 0, gateCollectWaypointYRed = 34;
        public double gateCollectWaypointXBlue = 0, gateCollectWaypointYBlue = -34;
        public double gateCollectLateralWeight = 2.8, gateShootAxialWeight = 1.2;
        public double gateCollectMinPower = 0.2;
        public double gateCollectOpenXRed = 7, gateCollectOpenYRed = 61, gateCollectOpenARed = Math.toRadians(116);
        public double gateCollectOpenXBlue = 7, gateCollectOpenYBlue = -61, gateCollectOpenABlue = Math.toRadians(-116);
        public double gateCollectXRed = 13, gateCollectYRed = 62, gateCollectARed = Math.toRadians(135);
        public double gateCollectXBlue = 13, gateCollectYBlue = -62, gateCollectABlue = Math.toRadians(-135);

    }
    public static class Shoot {
        public PathParams.HeadingLerpType preloadHeadingLerp = PathParams.HeadingLerpType.TANGENT;
        public double waypointTolX = 3, waypointTolY = 3, waypointTolA = Math.toRadians(5);
        public Tolerance waypointTol = new Tolerance(waypointTolX, waypointTolY, waypointTolA);
        public double waypointSlowDown = 0.3;

        public double maxFirstShootHeadingPower = 0.5;
        // shooting positions
        public double shootNearXRed = -15, shootNearYRed = 23, shootNearXBlue = -15, shootNearYBlue = -23;
        public double shootNearLastXRed = -25, shootNearLastYRed = 12, shootNearLastXBlue = -25, shootNearLastYBlue = -12;
        public double shootMidLastXRed = -23, shootMidLastYRed = 15, shootMidLastXBlue = -23, shootMidLastYBlue = -15;
        public double shootMidXRed = -8, shootMidYRed = 18.5, shootMidXBlue = -8, shootMidYBlue = -18.5;
        public double shootFarXRed = 54, shootFarYRed = 16, shootFarXBlue = 54, shootFarYBlue = -16;

        // custom shooting angles
        public double shootNearSetup1ARed = Math.toRadians(80), shootNearSetup2ARed = Math.toRadians(60), shootNearSetupGateARed = Math.toRadians(80), shootNearSetup3ARed = Math.toRadians(60), shootNearSetupLoadingARed = Math.toRadians(60);
        public double shootFarSetup1ARed = Math.toRadians(180), shootFarSetup2ARed = Math.toRadians(170), shootFarSetup3ARed = Math.toRadians(150), shootFarSetupLoadingARed = Math.toRadians(95);
        public double shootNearSetup1ABlue = Math.toRadians(-80), shootNearSetup2ABlue = Math.toRadians(-70), shootNearSetupGateABlue = Math.toRadians(-80), shootNearSetup3ABlue = Math.toRadians(-60), shootNearSetupLoadingABlue = Math.toRadians(-60);
        public double shootFarSetup1ABlue = Math.toRadians(-180), shootFarSetup2ABlue = Math.toRadians(-170), shootFarSetup3ABlue = Math.toRadians(-150), shootFarSetupLoadingABlue = Math.toRadians(-95);

        // shooting path waypoints to not hit other balls
        public double shootFar1WaypointXRed = 0, shootFar1WaypointYRed = 24, shootFar1WaypointARed = Math.toRadians(140);
        public double shootFar2WaypointXRed = 24, shootFar2WaypointYRed = 24, shootFar2WaypointARed = Math.toRadians(150);

        public double shootFar1WaypointXBlue = 0, shootFar1WaypointYBlue = -24, shootFar1WaypointABlue = Math.toRadians(-140);
        public double shootFar2WaypointXBlue = 24, shootFar2WaypointYBlue = -24, shootFar2WaypointABlue = Math.toRadians(-150);
    }

    public static class Misc {
        public double startNearXRed = -61.5, startNearYRed = 41.1, startNearARed = 0,
                startNearXBlue = -61.5, startNearYBlue = -41.1, startNearABlue = 0;
        public double startFarXRed = 60.8, startFarYRed = 15.4, startFarARed = Math.toRadians(180),
                startFarXBlue = 60.85, startFarYBlue = -19.2, startFarABlue = Math.toRadians(-180);
        public double gate1XRed = -3, gate2XRed = 5, gateYRed = 57;
        public double gate1XBlue = -3, gate2XBlue = 5, gateYBlue = -57;
        public double gateARed = Math.toRadians(90), gateABlue = Math.toRadians(-90);
        public double preGateClearance = 5, preGateXOffset = -5;
        public double preGateTolX = 3.5, preGateTolY = 3.5, preGateTolHeading = 7;
        public Tolerance preGateTol = new Tolerance(preGateTolX, preGateTolY, preGateTolHeading);
        public double gateCollectWaypointMinPower = 0.7, gateMinPower = 0.5;
        public double parkNearXRed = -2, parkNearYRed = 36, parkNearARed = Math.toRadians(45);
        public double parkNearXBlue = -2, parkNearYBlue = -36, parkNearABlue = Math.toRadians(-45);
        public double parkFarXRed = 50, parkFarYRed = 30, parkFarARed = Math.toRadians(135);
        public double parkFarXBlue = 50, parkFarYBlue = -30, parkFarABlue = Math.toRadians(-135);
        public double velToStopApplyingResistiveStopPowers = 10;
    }
    public static class TimeConstraints {
        public double gateOpeningWait = 0.5;
        public double gateCollectMaxTime = 1;
        public double cornerCollectMaxTime = 1.9;
        public double minShootTime = 0.5;
        public double parkStartTime = 29.5;
        public double stopEverythingTime = 35;
        public double postIntakeTime = 0.4, loadingSlowIntakeTime = 1;

    }
}

