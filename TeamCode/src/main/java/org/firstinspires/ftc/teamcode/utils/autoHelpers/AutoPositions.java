package org.firstinspires.ftc.teamcode.utils.autoHelpers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
public class AutoPositions {
    private final MecanumDrive drive;

    // red positions
    public static Pose2d redFarShootingPosition = new Pose2d(50, 10, Math.toRadians(180));
    public static Pose2d redCloseShootingPosition = new Pose2d(-40, 40, Math.toRadians(55));
    public static Pose2d redFirstLine = new Pose2d(-14, 26, Math.toRadians(90));
    public static Pose2d redSecondLine = new Pose2d(12, 26, Math.toRadians(97));
    public static Pose2d redThirdLine = new Pose2d(32, 26, Math.toRadians(90));
    public static Pose2d redApproachHP = new Pose2d(44, 68, Math.toRadians(45));
    public static Pose2d redGate1Left = new Pose2d(-8, 50, Math.toRadians(90));
    public static Pose2d redGate1Right = new Pose2d(0, 50, Math.toRadians(90));
    public static Pose2d redGate2 = new Pose2d(-4, 56, Math.toRadians(90));
    public static Pose2d redCloseEnd = new Pose2d(0, 45, Math.toRadians(0));
    public static Pose2d redFarEnd = new Pose2d(20, 45, Math.toRadians(180));

    // blue positions
    public final Pose2d blueFarShootingPosition = new Pose2d(50, -10, Math.toRadians(180));
    public final Pose2d blueCloseShootingPosition = new Pose2d(-34, -34, Math.toRadians(-135));
    public final Pose2d blueFirstLine = new Pose2d(-13, -24, Math.toRadians(-90));
    public final Pose2d blueSecondLine = new Pose2d(12, -24, Math.toRadians(-90));
    public final Pose2d blueThirdLine = new Pose2d(34.5, -24, Math.toRadians(-90));
    public final Pose2d blueApproachHP = new Pose2d(44, -68, Math.toRadians(-45));

    public static Pose2d blueGate1Left = new Pose2d(-8, -50, Math.toRadians(90));
    public static Pose2d blueGate1Right = new Pose2d(0, 50, Math.toRadians(90));
    public static Pose2d blueGate2 = new Pose2d(-4, 56, Math.toRadians(90));
    public final Pose2d blueCloseEnd = new Pose2d(0, -45, Math.toRadians(0));
    public final Pose2d blueFarEnd = new Pose2d(20, -45, Math.toRadians(180));

    public AutoPositions(MecanumDrive drive) {
        this.drive = drive;
    }

    public Action redDriveCloseShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .lineToXLinearHeading(-35, Math.toRadians(55));
        return moveOffWall.build();
    }
    public Action redCollectAndShootFirstLine(boolean startingClose, boolean shootingClose, boolean releaseGate) {
        double driveThroughY = 55;
        Action driveToFirstLine = drive.actionBuilder(startingClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(redFirstLine, Math.toRadians(90)).build();
        Action driveThroughFirstLine = drive.actionBuilder(redFirstLine)
                .lineToY(driveThroughY).build();
        Action possiblyOpenGate = releaseGate ?
                openRedGateFromLine(new Pose2d(redFirstLine.position.x, driveThroughY, redFirstLine.heading.toDouble()), 1) :
                packet -> false;
        Action driveToShoot = drive.actionBuilder(redFirstLine)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(
                        shootingClose ? redCloseShootingPosition : redFarShootingPosition,
                        shootingClose ? Math.toRadians(180) : Math.toRadians(0)
                ).build();

        return new SequentialAction(
                new TimedAction(driveToFirstLine, 1.5),
                new TimedAction(driveThroughFirstLine, 1),
                new TimedAction(possiblyOpenGate, 2.5),
                new TimedAction(driveToShoot, 2)
        );
    }
    public Action redCollectAndShootSecondLine(boolean startingClose, boolean shootingClose, boolean releaseGate) {
        double driveThroughY = 62;
        Action driveToLine = drive.actionBuilder(startingClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(redSecondLine, Math.toRadians(90)).build();
        Action driveThroughLine = drive.actionBuilder(redSecondLine)
                .lineToY(driveThroughY).build();
        Action possiblyOpenGate = releaseGate ?
                openRedGateFromLine(new Pose2d(redFirstLine.position.x, driveThroughY, redFirstLine.heading.toDouble()), 2) :
                packet -> false;
        Action driveToShoot = drive.actionBuilder(redSecondLine)
                .setTangent(Math.toRadians(-110))
                .splineToLinearHeading(
                        shootingClose ? redCloseShootingPosition : redFarShootingPosition,
                        shootingClose ? Math.toRadians(180) : Math.toRadians(0)
                ).build();

        return new SequentialAction(
                new TimedAction(driveToLine, 2.5),
                new TimedAction(driveThroughLine, 1.5),
                new TimedAction(possiblyOpenGate, 2.5),
                new TimedAction(driveToShoot, 2.75)
        );
    }
    public Action redCollectAndShootThirdLine(boolean startingClose, boolean shootingClose, boolean releaseGate) {
        double driveThroughY = 62;
        Action driveToLine = drive.actionBuilder(startingClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redThirdLine, Math.toRadians(90)).build();
        Action driveThroughLine = drive.actionBuilder(redThirdLine)
                .lineToY(driveThroughY).build();
        Action possiblyOpenGate = releaseGate ?
                openRedGateFromLine(new Pose2d(redFirstLine.position.x, driveThroughY, redFirstLine.heading.toDouble()), 3) :
                packet -> false;
        Action driveToShoot = drive.actionBuilder(redThirdLine)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(
                        shootingClose ? redCloseShootingPosition : redFarShootingPosition,
                        shootingClose ? Math.toRadians(180) : Math.toRadians(0)
                ).build();

        return new SequentialAction(
                new TimedAction(driveToLine, 3.5),
                new TimedAction(driveThroughLine, 2),
                new TimedAction(possiblyOpenGate, 2.5),
                new TimedAction(driveToShoot, 3.25)
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
            moveOffLine = drive.actionBuilder(redCloseShootingPosition)
                    .splineToLinearHeading(redCloseEnd, Math.toRadians(90));
        else
            moveOffLine = drive.actionBuilder(redFarShootingPosition)
                    .splineToLinearHeading(redFarEnd, Math.toRadians(90));

        return moveOffLine.build();
    }

    public Action redCollectAndShootHumanPlayer(boolean startingClose, boolean shootingClose) {
        return drive.actionBuilder(startingClose ? redCloseShootingPosition : redFarShootingPosition)
                .splineTo(redApproachHP.position, redApproachHP.heading)
                .lineToX(65)
                .waitSeconds(0.5)
                .setTangent(0)
                .splineToLinearHeading(
                        shootingClose ? redCloseShootingPosition : redFarShootingPosition,
                        shootingClose ? Math.toRadians(180) : Math.toRadians(0)
                ).build();
    }

    //blue actions
    public Action blueDriveCloseShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .lineToXLinearHeading(-35, Math.toRadians(-125));
        return moveOffWall.build();
    }
    public Action blueCollectAndShootFirstLine(boolean startingClose, boolean shootingClose, boolean releaseGate) {
        double driveThroughY = -55;
        Action driveToLine = drive.actionBuilder(startingClose ? blueCloseShootingPosition : blueFarShootingPosition)
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
                        shootingClose ? blueCloseShootingPosition : blueFarShootingPosition,
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
        Action driveToLine = drive.actionBuilder(startingClose ? blueCloseShootingPosition : blueFarShootingPosition)
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
                        shootingClose ? blueCloseShootingPosition : blueFarShootingPosition,
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

        Action driveToLine = drive.actionBuilder(startingClose ? blueCloseShootingPosition : blueFarShootingPosition)
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
                        shootingClose ? blueCloseShootingPosition : blueFarShootingPosition,
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
            moveOffLine = drive.actionBuilder(blueCloseShootingPosition)
                    .setTangent(0)
                    .splineToLinearHeading(blueCloseEnd, Math.toRadians(-90));
        else
            moveOffLine = drive.actionBuilder(blueFarShootingPosition)
                    .setTangent(0)
                    .splineToLinearHeading(blueFarEnd, Math.toRadians(-90));

        return moveOffLine.build();
    }
    public Action blueCollectAndShootHumanPlayer(boolean startingClose, boolean shootingClose) {
        TrajectoryActionBuilder hpShot = drive.actionBuilder(startingClose ? blueCloseShootingPosition : blueFarShootingPosition)
                .splineTo(blueApproachHP.position, blueApproachHP.heading)
                .waitSeconds(0.25)
                .lineToX(-65)
                .waitSeconds(0.25)
                .setTangent(0)
                .splineToLinearHeading(
                        shootingClose ? blueCloseShootingPosition : blueFarShootingPosition,
                        shootingClose ? Math.toRadians(180) : Math.toRadians(0)
                );
        return hpShot.build();
    }
}
