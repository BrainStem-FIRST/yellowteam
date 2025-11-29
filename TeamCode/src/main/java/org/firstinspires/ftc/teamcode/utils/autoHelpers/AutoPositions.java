package org.firstinspires.ftc.teamcode.utils.autoHelpers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.Arrays;

@Config
public class AutoPositions {
    private final MecanumDrive drive;

    // red positions
    public static Pose2d redFarShootingPose = new Pose2d(50, 10, Math.toRadians(180));
    public static Pose2d redCloseShootPose = new Pose2d(-13, 22, Math.toRadians(90));
    public static Pose2d redCloseShootPose2 = new Pose2d(-16, 22, Math.toRadians(90));
    public static Pose2d redFirstLine = new Pose2d(-14.5, 26, Math.toRadians(90));
    public static Pose2d redSecondLine = new Pose2d(12, 26, Math.toRadians(97));
    public static Pose2d redThirdLine = new Pose2d(48, 24, Math.toRadians(90));
    public static Pose2d redThirdLine2 = new Pose2d(36, 62, Math.toRadians(90));
    public static Pose2d redApproachHP = new Pose2d(44, 68, Math.toRadians(45));
    public static Pose2d redGate1Left = new Pose2d(-8, 50, Math.toRadians(90));
    public static Pose2d redGate1Right = new Pose2d(0, 50, Math.toRadians(90));
    public static Pose2d redGate2 = new Pose2d(-4, 56, Math.toRadians(90));
    public static Pose2d redCloseEnd = new Pose2d(0, 45, Math.toRadians(0));
    public static Pose2d redFarEnd = new Pose2d(20, 45, Math.toRadians(180));

    // blue positions
    public static Pose2d blueFarShootingPosition = new Pose2d(50, -10, Math.toRadians(180));
    public static Pose2d blueCloseShootPose = new Pose2d(-24, -24, Math.toRadians(-135));
    public static Pose2d blueFirstLine = new Pose2d(-13, -24, Math.toRadians(-90));
    public static Pose2d blueSecondLine = new Pose2d(12, -24, Math.toRadians(-90));
    public static Pose2d blueThirdLine = new Pose2d(34.5, -24, Math.toRadians(-90));
    public static Pose2d blueApproachHP = new Pose2d(44, -68, Math.toRadians(-45));

    public static Pose2d blueGate1Left = new Pose2d(-8, -50, Math.toRadians(90));
    public static Pose2d blueGate1Right = new Pose2d(0, 50, Math.toRadians(90));
    public static Pose2d blueGate2 = new Pose2d(-4, 56, Math.toRadians(90));
    public static Pose2d blueCloseEnd = new Pose2d(0, -45, Math.toRadians(0));
    public static Pose2d blueFarEnd = new Pose2d(20, -45, Math.toRadians(180));

    public AutoPositions(MecanumDrive drive) {
        this.drive = drive;
    }

    public Action redDriveCloseShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .strafeToLinearHeading(redCloseShootPose.position, redCloseShootPose.heading.toDouble());
        return new TimedAction(moveOffWall.build(), 2.5);
    }
    public Action redDriveFarShootingPose(Pose2d startPose) {
        return drive.actionBuilder(startPose)
                .lineToXLinearHeading(55, Math.toRadians(140))
                .build();
    }
    public Action collectAndShoot(Alliance alliance, int lineToCollect, boolean releaseGate, Pose2d shootPose, double shootToCollectTangent, double collectApproachTangent, Pose2d collectApproachPose, double driveThroughLineY, double collectToShootTangent, double shootApproachTangent, double[] maxTimes, AutoRobotStatus robotStatus) {
        if (maxTimes.length != 3)
            throw new IllegalArgumentException("need to pass in 3 max times - passed in " + maxTimes.length + ": " + Arrays.toString(maxTimes));

        boolean onRedAlliance = alliance == Alliance.RED;
        if (!onRedAlliance) {
            shootToCollectTangent *= -1;
            collectApproachTangent *= -1;
            driveThroughLineY *= -1;
            collectToShootTangent *= -1;
            shootApproachTangent *= -1;

        }
        Action driveToLine = drive.actionBuilder(shootPose)
                .setTangent(shootToCollectTangent)
                .splineToLinearHeading(collectApproachPose, collectApproachTangent)
                .build();
        Action driveThroughLine = drive.actionBuilder(collectApproachPose)
                .lineToY(driveThroughLineY)
                .build();

        Action possiblyOpenGate;
        Pose2d gateEndPose;
        if(releaseGate) {
            if (onRedAlliance) {
                possiblyOpenGate = openRedGateFromLine(new Pose2d(collectApproachPose.position.x, driveThroughLineY, collectApproachPose.heading.toDouble()), lineToCollect);
                gateEndPose = redGate2;
            }
            else {
                possiblyOpenGate = openBlueGateFromLine(new Pose2d(collectApproachPose.position.x, driveThroughLineY, collectApproachPose.heading.toDouble()), lineToCollect);
                gateEndPose = blueGate2;
            }
        }
        else {
            possiblyOpenGate = packet -> false;
            gateEndPose = collectApproachPose;
        }

        Action driveToShoot = drive.actionBuilder(gateEndPose)
                .setTangent(collectToShootTangent)
                .splineToLinearHeading(
                        shootPose,
                        shootApproachTangent
                )
                .build();

        robotStatus.preCollecting = true;
        return new SequentialAction(
                new TimedAction(driveToLine, 1.5),
                telemetryPacket -> {robotStatus.cycle(); return false;},
                new TimedAction(driveThroughLine, 1),
                telemetryPacket -> {robotStatus.cycle(); return false;},
                new TimedAction(possiblyOpenGate, 2.5),
                new TimedAction(driveToShoot, 2),
                telemetryPacket -> {robotStatus.cycle(); return false;}
        );
    }

    public Action redCollectAndShootFirstLine(boolean startingClose, boolean shootingClose, boolean releaseGate) {
        double driveThroughY = 55;
        Action driveToLine = drive.actionBuilder(startingClose ? redCloseShootPose : redFarShootingPose)
                .strafeToLinearHeading(new Vector2d(redFirstLine.position.x, driveThroughY), Math.toRadians(90))
                .build();
        Action possiblyOpenGate = releaseGate ?
                openRedGateFromLine(new Pose2d(redFirstLine.position.x, driveThroughY, redFirstLine.heading.toDouble()), 1) :
                packet -> false;
        Action driveToShoot = drive.actionBuilder(releaseGate ? redGate2 : redFirstLine)
                .setTangent(shootingClose ? Math.toRadians(-135) : Math.toRadians(-45))
                .splineToLinearHeading(
                        shootingClose ? redCloseShootPose : redFarShootingPose,
                        shootingClose ? Math.toRadians(180) : Math.toRadians(0)
                ).build();

        return new SequentialAction(
                new CustomEndAction(driveToLine, () -> drive.localizer.getPose().position.y >= driveThroughY - 1, 2.5),
                new TimedAction(possiblyOpenGate, 2.5),
                new TimedAction(driveToShoot, 1.7)
        );
    }
    public Action redCollectAndShootSecondLine(boolean startingClose, boolean shootingClose, boolean releaseGate) {
        double driveThroughY = 62;
        Action driveToLine = drive.actionBuilder(startingClose ? redCloseShootPose : redFarShootingPose)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(redSecondLine, Math.toRadians(90))
                .lineToY(driveThroughY).build();
        Action possiblyOpenGate = releaseGate ?
                openRedGateFromLine(new Pose2d(redFirstLine.position.x, driveThroughY, redFirstLine.heading.toDouble()), 2) :
                packet -> false;
        Action driveToShoot = drive.actionBuilder(redSecondLine)
                .setTangent(Math.toRadians(-110))
                .splineToLinearHeading(
                        shootingClose ? redCloseShootPose2 : redFarShootingPose,
                        shootingClose ? Math.toRadians(180) : Math.toRadians(0)
                ).build();

        return new SequentialAction(
//                new TimedAction(driveToLine, 4),
                new CustomEndAction(driveToLine, () -> drive.localizer.getPose().position.y >= driveThroughY - 1, 4),
                new TimedAction(possiblyOpenGate, 2.5),
                new TimedAction(driveToShoot, 2.75)
        );
    }
    public Action redCollectAndShootThirdLine(boolean startingClose, boolean shootingClose, boolean releaseGate) {
        Action driveToLine = drive.actionBuilder(startingClose ? redCloseShootPose2 : redFarShootingPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redThirdLine, Math.toRadians(90))
                .strafeToLinearHeading(redThirdLine2.position, Math.toRadians(90)).build();
        Action possiblyOpenGate = releaseGate ?
                openRedGateFromLine(new Pose2d(redFirstLine.position.x, redThirdLine2.position.y, redFirstLine.heading.toDouble()), 3) :
                packet -> false;
        Action driveToShoot = drive.actionBuilder(new Pose2d(redThirdLine.position.x, redThirdLine2.position.y, Math.toRadians(90)))
                .strafeToLinearHeading(
                        shootingClose ? redCloseShootPose2.position : redFarShootingPose.position,
                        shootingClose ? redCloseShootPose2.heading.toDouble() : redFarShootingPose.heading.toDouble()
                ).build();

        return new SequentialAction(
                new CustomEndAction(driveToLine, () -> drive.localizer.getPose().position.y >= redThirdLine2.position.y - 1, 5.5).setEndFunction(drive::stop),
                new TimedAction(possiblyOpenGate, 2.5).setEndFunction(drive::stop),
                new TimedAction(driveToShoot, 2.25).setEndFunction(drive::stop)
        );
    }
    public Action openRedGateFromLine(Pose2d startPose, int lineJustCollected) {
        if (lineJustCollected == 1) {
            return drive.actionBuilder(startPose)
                    .splineToLinearHeading(redGate1Left, Math.toRadians(90))
                    .splineToLinearHeading(redGate2, Math.toRadians(90)).build();
        }
        return drive.actionBuilder(startPose)
                .splineToLinearHeading(redGate1Right, Math.toRadians(90))
                .splineToLinearHeading(redGate2, Math.toRadians(90)).build();
    }
    public Action redMoveOffLine(boolean startingClose) {
        TrajectoryActionBuilder moveOffLine;
        if (startingClose)
            moveOffLine = drive.actionBuilder(redCloseShootPose)
                    .splineToLinearHeading(redCloseEnd, Math.toRadians(90));
        else
            moveOffLine = drive.actionBuilder(redFarShootingPose)
                    .splineToLinearHeading(redFarEnd, Math.toRadians(90));

        return moveOffLine.build();
    }

    public Action redCollectAndShootLoadingZone(boolean startingClose, boolean shootingClose) {
        return drive.actionBuilder(startingClose ? redCloseShootPose : redFarShootingPose)
                .splineTo(redApproachHP.position, redApproachHP.heading)
                .lineToX(65)
                .waitSeconds(0.5)
                .setTangent(0)
                .splineToLinearHeading(
                        shootingClose ? redCloseShootPose : redFarShootingPose,
                        shootingClose ? Math.toRadians(180) : Math.toRadians(0)
                ).build();
    }

    //blue actions
    public Action blueDriveCloseShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .lineToXLinearHeading(-35, Math.toRadians(-125));
        return moveOffWall.build();
    }
    public Action blueDriveFarShootingPose(Pose2d startPose) {
        return drive.actionBuilder(startPose)
                .lineToXLinearHeading(55, Math.toRadians(-140))
                .build();
    }
    public Action blueCollectAndShootFirstLine(boolean startingClose, boolean shootingClose, boolean releaseGate) {
        double driveThroughY = -55;
        Action driveToLine = drive.actionBuilder(startingClose ? blueCloseShootPose : blueFarShootingPosition)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(blueFirstLine, Math.toRadians(90)).build();
        Action driveThroughLine = drive.actionBuilder(blueFirstLine)
                .lineToY(driveThroughY).build();
        Action possiblyOpenGate = releaseGate ?
                openBlueGateFromLine(new Pose2d(redFirstLine.position.x, driveThroughY, redFirstLine.heading.toDouble()), 1) :
                packet -> false;
        Action driveToShoot = drive.actionBuilder(blueFirstLine)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(
                        shootingClose ? blueCloseShootPose : blueFarShootingPosition,
                        shootingClose ? Math.toRadians(180) : Math.toRadians(0)
                ).build();

        return new SequentialAction(
                new TimedAction(driveToLine, 1),
                new TimedAction(driveThroughLine, 1.5),
                new TimedAction(possiblyOpenGate, 2.5),
                new TimedAction(driveToShoot, 2)
        );
    }
    public Action blueCollectAndShootSecondLine(boolean startingClose, boolean shootingClose, boolean releaseGate) {
        double driveThroughY = -62;
        Action driveToLine = drive.actionBuilder(startingClose ? blueCloseShootPose : blueFarShootingPosition)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(blueSecondLine, Math.toRadians(90)).build();
        Action driveThroughLine = drive.actionBuilder(blueSecondLine)
                .lineToY(driveThroughY).build();
        Action possiblyOpenGate = releaseGate ?
                openBlueGateFromLine(new Pose2d(redFirstLine.position.x, driveThroughY, redFirstLine.heading.toDouble()), 1) :
                packet -> false;
        Action driveToShoot = drive.actionBuilder(blueSecondLine)
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(
                        shootingClose ? blueCloseShootPose : blueFarShootingPosition,
                        shootingClose ? Math.toRadians(180) : Math.toRadians(0)
                ).build();

        return new SequentialAction(
                new TimedAction(driveToLine, 1.5),
                new TimedAction(driveThroughLine, 1.75),
                new TimedAction(possiblyOpenGate, 2.5),
                new TimedAction(driveToShoot, 2)
        );
    }
    public Action blueCollectAndShootThirdLine(boolean startingClose, boolean shootingClose, boolean releaseGate) {
        double driveThroughY = -62;

        Action driveToLine = drive.actionBuilder(startingClose ? blueCloseShootPose : blueFarShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(blueThirdLine, Math.toRadians(-90)).build();
        Action driveThroughLine = drive.actionBuilder(blueThirdLine)
                .lineToY(driveThroughY).build();
        Action possiblyOpenGate = releaseGate ?
                openBlueGateFromLine(new Pose2d(redFirstLine.position.x, driveThroughY, redFirstLine.heading.toDouble()), 1) :
                packet -> false;
        Action driveToShoot = drive.actionBuilder(blueThirdLine)
                .setTangent(Math.toRadians(20))
                .splineToLinearHeading(
                        shootingClose ? blueCloseShootPose : blueFarShootingPosition,
                        shootingClose ? Math.toRadians(180) : Math.toRadians(0)
                ).build();

        return new SequentialAction(
                new TimedAction(driveToLine, 2.5),
                new TimedAction(driveThroughLine, 2.25),
                new TimedAction(possiblyOpenGate, 2.5),
                new TimedAction(driveToShoot, 3.75)
        );
    }
    public Action openBlueGateFromLine(Pose2d startPose, int lineJustCollected) {
        if (lineJustCollected == 1) {
            return drive.actionBuilder(startPose)
                    .splineToLinearHeading(blueGate1Left, Math.toRadians(-90))
                    .splineToLinearHeading(blueGate2, Math.toRadians(-90)).build();
        }
        return drive.actionBuilder(startPose)
                .splineToLinearHeading(blueGate1Right, Math.toRadians(-90))
                .splineToLinearHeading(blueGate2, Math.toRadians(-90)).build();
    }
    public Action blueMoveOffLine(boolean isClose) {
        TrajectoryActionBuilder moveOffLine;
        if (isClose)
            moveOffLine = drive.actionBuilder(blueCloseShootPose)
                    .setTangent(0)
                    .splineToLinearHeading(blueCloseEnd, Math.toRadians(-90));
        else
            moveOffLine = drive.actionBuilder(blueFarShootingPosition)
                    .setTangent(0)
                    .splineToLinearHeading(blueFarEnd, Math.toRadians(-90));

        return moveOffLine.build();
    }
    public Action blueCollectAndShootLoadingZone(boolean startingClose, boolean shootingClose) {
        TrajectoryActionBuilder hpShot = drive.actionBuilder(startingClose ? blueCloseShootPose : blueFarShootingPosition)
                .splineTo(blueApproachHP.position, blueApproachHP.heading)
                .waitSeconds(0.25)
                .lineToX(-65)
                .waitSeconds(0.25)
                .setTangent(0)
                .splineToLinearHeading(
                        shootingClose ? blueCloseShootPose : blueFarShootingPosition,
                        shootingClose ? Math.toRadians(180) : Math.toRadians(0)
                );
        return hpShot.build();
    }
}
