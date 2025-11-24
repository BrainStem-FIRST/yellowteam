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

    //Red positions
    public static Pose2d redFarShootingPosition = new Pose2d(50, 10, Math.toRadians(180));
    public static Pose2d redCloseShootingPosition = new Pose2d(-40, 40, Math.toRadians(55));
    public static Pose2d redFirstLine = new Pose2d(-14, 26, Math.toRadians(90));
    public static Pose2d redSecondLine = new Pose2d(12, 26, Math.toRadians(97));
    public static Pose2d redThirdLine = new Pose2d(32, 26, Math.toRadians(90));
    public static Pose2d redApproachHP = new Pose2d(35, 68, Math.toRadians(45));
    public static Pose2d redGate1Left = new Pose2d(-8, 50, Math.toRadians(90));
    public static Pose2d redGate1Right = new Pose2d(0, 50, Math.toRadians(90));
    public static Pose2d redGate2 = new Pose2d(-4, 56, Math.toRadians(90));
    public static Pose2d redCloseEnd = new Pose2d(0, 45, Math.toRadians(0));
    public static Pose2d redFarEnd = new Pose2d(20, 45, Math.toRadians(180));

    //blue positions
    public final Pose2d blueFarShootingPosition = new Pose2d(50, -10, Math.toRadians(180));
    public final Pose2d blueCloseShootingPosition = new Pose2d(-34, -34, Math.toRadians(-135));
    public final Pose2d blueFirstLine = new Pose2d(-13, -24, Math.toRadians(-90));
    public final Pose2d blueSecondLine = new Pose2d(12, -24, Math.toRadians(-90));
    public final Pose2d blueThirdLine = new Pose2d(34.5, -24, Math.toRadians(-90));
    public final Pose2d blueApproachHP = new Pose2d(35, -68, Math.toRadians(-135));

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
    public Action redCollectAndShootFirstLine(boolean startingClose, boolean releaseGate) {
        double gateY = 55;
        Action driveToFirstLine = drive.actionBuilder(startingClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(redFirstLine, Math.toRadians(90)).build();
        Action driveThroughFirstLine = drive.actionBuilder(redFirstLine)
                .lineToY(gateY).build();
        Action possiblyOpenGate = releaseGate ?
                openRedGateFromLine(new Pose2d(redFirstLine.position.x, gateY, redFirstLine.heading.toDouble()), 1) :
                packet -> false;
        Action driveToShoot = drive.actionBuilder(redFirstLine)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(redCloseShootingPosition, Math.toRadians(180)).build();

        Action maxTimeDriveToLine = new TimedAction(driveToFirstLine, 1.5);
        Action maxTimeDriveThroughLine = new TimedAction(driveThroughFirstLine, 1);
        Action maxTimePossiblyOpenGate = new TimedAction(possiblyOpenGate, 2.5);
        Action maxTimeDriveToShoot = new TimedAction(driveToShoot, 2);

        return new SequentialAction(
                maxTimeDriveToLine,
                maxTimeDriveThroughLine,
                maxTimePossiblyOpenGate,
                maxTimeDriveToShoot
        );
    }
    public Action redCollectAndShootSecondLine(boolean startingClose, boolean releaseGate) {
        double gateY = 62;
        Action driveToLine = drive.actionBuilder(startingClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(redSecondLine, Math.toRadians(90)).build();
        Action driveThroughLine = drive.actionBuilder(redSecondLine)
                .lineToY(gateY).build();
        Action possiblyOpenGate = releaseGate ?
                openRedGateFromLine(new Pose2d(redFirstLine.position.x, gateY, redFirstLine.heading.toDouble()), 2) :
                packet -> false;
        Action driveToShoot = drive.actionBuilder(redSecondLine)
                .setTangent(Math.toRadians(-110))
                .splineToLinearHeading(redCloseShootingPosition, Math.toRadians(180)).build();

        Action maxTimeDriveToLine = new TimedAction(driveToLine, 2.5);
        Action maxTimeDriveThroughLine = new TimedAction(driveThroughLine, 1.5);
        Action maxTimePossiblyOpenGate = new TimedAction(possiblyOpenGate, 2.5);
        Action maxTimeDriveToShoot = new TimedAction(driveToShoot, 2.75);

        return new SequentialAction(
                maxTimeDriveToLine,
                maxTimeDriveThroughLine,
                maxTimePossiblyOpenGate,
                maxTimeDriveToShoot
        );
    }
    public Action redCollectAndShootThirdLine(boolean startingClose, boolean releaseGate) {
        double gateY = 62;
        Action firstLineShot = drive.actionBuilder(startingClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redThirdLine, Math.toRadians(90)).build();
        Action secondPath = drive.actionBuilder(redThirdLine)
                .lineToY(gateY).build();
        Action possiblyOpenGate = releaseGate ?
                openRedGateFromLine(new Pose2d(redFirstLine.position.x, gateY, redFirstLine.heading.toDouble()), 2) :
                packet -> false;
        Action thirdPath = drive.actionBuilder(redThirdLine)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(redCloseShootingPosition, Math.toRadians(180)).build();

        Action maxTimeFirstPath = new TimedAction(firstLineShot, 3.5);
        Action maxTimeSecondPath = new TimedAction(secondPath, 2);
        Action maxTimePossiblyOpenGate = new TimedAction(possiblyOpenGate, 2.5);
        Action maxTimeThirdPath = new TimedAction(thirdPath, 3.25);

        return new SequentialAction(
                maxTimeFirstPath,
                maxTimeSecondPath,
                maxTimePossiblyOpenGate,
                maxTimeThirdPath
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

    public Action humanPlayerShots(boolean isClose) {
        TrajectoryActionBuilder hpShot = drive.actionBuilder(isClose ? redCloseShootingPosition : redFarShootingPosition)
                .splineTo(redApproachHP.position, redApproachHP.heading)
                .waitSeconds(0.5)
                .lineToX(65)
                .waitSeconds(0.5)
                .setTangent(0)
                .splineToLinearHeading(isClose ? redCloseShootingPosition : redFarShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));
        return hpShot.build();
    }

    //blue actions
    public Action blueDriveCloseShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .lineToXLinearHeading(-35, Math.toRadians(-125));
        return moveOffWall.build();
    }
    public Action blueCollectAndShootFirstLine(boolean isClose, boolean releaseGate) {
        Action firstLineShot = drive.actionBuilder(isClose ? blueCloseShootingPosition : blueFarShootingPosition)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(blueFirstLine, Math.toRadians(90)).build();
        Action secondPath = drive.actionBuilder(blueFirstLine)
                .lineToY(-60).build();
        Action thirdPath = drive.actionBuilder(blueFirstLine)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(blueCloseShootingPosition, Math.toRadians(180)).build();

        Action maxTimeFirstPath = new TimedAction(firstLineShot, 1);
        Action maxTimeSecondPath = new TimedAction(secondPath, 1.5);
        Action maxTimeThirdPath = new TimedAction(thirdPath, 2);

        return new SequentialAction(
                maxTimeFirstPath,
                maxTimeSecondPath,
                maxTimeThirdPath
        );
    }
    public Action blueCollectAndShootSecondLine(boolean startingClose, boolean releaseGate) {
        Action firstLineShot = drive.actionBuilder(startingClose ? blueCloseShootingPosition : blueFarShootingPosition)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(blueSecondLine, Math.toRadians(90)).build();
        Action secondPath = drive.actionBuilder(blueSecondLine)
                .lineToY(-63).build();
        Action thirdPath = drive.actionBuilder(blueSecondLine)
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(blueCloseShootingPosition, Math.toRadians(180)).build();

        Action maxTimeFirstPath = new TimedAction(firstLineShot, 1.5);
        Action maxTimeSecondPath = new TimedAction(secondPath, 1.75);
        Action maxTimeThirdPath = new TimedAction(thirdPath, 2);

        return new SequentialAction(
                maxTimeFirstPath,
                maxTimeSecondPath,
                maxTimeThirdPath
        );
    }
    public Action blueCollectAndShootThirdLine(boolean startingClose, boolean releaseGate) {
        Action firstLineShot = drive.actionBuilder(startingClose ? blueCloseShootingPosition : blueFarShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(blueThirdLine, Math.toRadians(-90)).build();
        Action secondPath = drive.actionBuilder(blueThirdLine)
                .lineToY(-63).build();
        Action thirdPath = drive.actionBuilder(blueThirdLine)
                .setTangent(Math.toRadians(20))
                .splineToLinearHeading(blueCloseShootingPosition, Math.toRadians(180)).build();

        Action maxTimeFirstPath = new TimedAction(firstLineShot, 2.5);
        Action maxTimeSecondPath = new TimedAction(secondPath, 2.25);
        Action maxTimeThirdPath = new TimedAction(thirdPath, 3.75);

        return new SequentialAction(
                maxTimeFirstPath,
                maxTimeSecondPath,
                maxTimeThirdPath
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
    public Action blueHumanPlayerShots(boolean isClose) {
        TrajectoryActionBuilder hpShot = drive.actionBuilder(isClose ? blueCloseShootingPosition : blueFarShootingPosition)
                .splineTo(blueApproachHP.position, blueApproachHP.heading)
                .waitSeconds(0.25)
                .lineToX(-65)
                .waitSeconds(0.25)
                .setTangent(0)
                .splineToLinearHeading(isClose ? blueCloseShootingPosition : blueFarShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));
        return hpShot.build();
    }
}
