package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class AutoPositions {
    private final MecanumDrive drive;

    //Red positions
    public final Pose2d redFarShootingPosition = new Pose2d(50, 10, Math.toRadians(180));
    public final Pose2d redCloseShootingPosition = new Pose2d(-40, 40, Math.toRadians(55));
    public final Pose2d redFirstLine = new Pose2d(-14, 26, Math.toRadians(90));
    public final Pose2d redSecondLine = new Pose2d(12, 26, Math.toRadians(97));
    public final Pose2d redThirdLine = new Pose2d(32, 26, Math.toRadians(90));
    public final Pose2d redApproachHP = new Pose2d(35, 68, Math.toRadians(45));
    public final Pose2d redGate = new Pose2d(12, 30, Math.toRadians(90));
    public final Pose2d redEnd = new Pose2d(0, 45, Math.toRadians(0));

    //blue positions
    public final Pose2d blueFarShootingPosition = new Pose2d(50, -10, Math.toRadians(180));
    public final Pose2d blueCloseShootingPosition = new Pose2d(-34, -34, Math.toRadians(-135));
    public final Pose2d blueFirstLine = new Pose2d(-13, -24, Math.toRadians(-90));
    public final Pose2d blueSecondLine = new Pose2d(12, -24, Math.toRadians(-90));
    public final Pose2d blueThirdLine = new Pose2d(34.5, -24, Math.toRadians(-90));
    public final Pose2d blueApproachHP = new Pose2d(35, -68, Math.toRadians(-135));
    public final Pose2d blueEnd = new Pose2d(0, -45, Math.toRadians(0));

    public AutoPositions(MecanumDrive drive) {
        this.drive = drive;
    }

    public Action driveToFarShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .splineToConstantHeading(redFarShootingPosition.position, redFarShootingPosition.heading);
        return moveOffWall.build();
    }

    public Action moveOffLine(boolean isClose) {
        TrajectoryActionBuilder moveOffLine = drive.actionBuilder(redCloseShootingPosition)
                .splineToLinearHeading(redEnd, Math.toRadians(90));
        return moveOffLine.build();
    }

    public Action driveCloseShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .lineToXLinearHeading(-35, Math.toRadians(55));
        return moveOffWall.build();
    }

    public Action firstLineShots(boolean isClose) {
        Action firstLineShot = drive.actionBuilder(isClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(redFirstLine, Math.toRadians(90)).build();
        Action secondPath = drive.actionBuilder(redFirstLine)
                .lineToY(55).build();
        Action thirdPath = drive.actionBuilder(redFirstLine)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(redCloseShootingPosition, Math.toRadians(180)).build();

        Action maxTimeFirstPath = new TimedAction(firstLineShot, 1.5);
        Action maxTimeSecondPath = new TimedAction(secondPath, 1);
        Action maxTimeThirdPath = new TimedAction(thirdPath, 2);

        return new SequentialAction(
                maxTimeFirstPath,
                maxTimeSecondPath,
                maxTimeThirdPath
        );
    }

    public Action secondLineShots(boolean isClose) {
        Action firstLineShot = drive.actionBuilder(isClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(redSecondLine, Math.toRadians(90)).build();
        Action secondPath = drive.actionBuilder(redSecondLine)
                .lineToY(62).build();
        Action thirdPath = drive.actionBuilder(redSecondLine)
                .setTangent(Math.toRadians(-110))
                .splineToLinearHeading(redCloseShootingPosition, Math.toRadians(180)).build();

        Action maxTimeFirstPath = new TimedAction(firstLineShot, 2.5);
        Action maxTimeSecondPath = new TimedAction(secondPath, 1.5);
        Action maxTimeThirdPath = new TimedAction(thirdPath, 2.75);

        return new SequentialAction(
                maxTimeFirstPath,
                maxTimeSecondPath,
                maxTimeThirdPath
        );
    }
    public Action secondLineToGate(boolean isClose) {
        Action firstLineShot = drive.actionBuilder(isClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(redSecondLine, Math.toRadians(90)).build();
        Action secondPath = drive.actionBuilder(redSecondLine)
                .lineToY(62).build();
        Action thirdPath = drive.actionBuilder(redSecondLine)
                .setTangent(Math.toRadians(-110))
                .splineTo(redGate.position, redGate.heading)
                .splineToLinearHeading(redCloseShootingPosition, Math.toRadians(180)).build();

        Action maxTimeFirstPath = new TimedAction(firstLineShot, 2.5);
        Action maxTimeSecondPath = new TimedAction(secondPath, 1.5);
        Action maxTimeThirdPath = new TimedAction(thirdPath, 2.75);

        return new SequentialAction(
                maxTimeFirstPath,
                maxTimeSecondPath,
                maxTimeThirdPath
        );
    }


    public Action thirdLineShots(boolean isClose) {
        Action firstLineShot = drive.actionBuilder(isClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redThirdLine, Math.toRadians(90)).build();
        Action secondPath = drive.actionBuilder(redThirdLine)
                .lineToY(60).build();
        Action thirdPath = drive.actionBuilder(redThirdLine)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(redCloseShootingPosition, Math.toRadians(180)).build();

        Action maxTimeFirstPath = new TimedAction(firstLineShot, 3.5);
        Action maxTimeSecondPath = new TimedAction(secondPath, 2);
        Action maxTimeThirdPath = new TimedAction(thirdPath, 3.25);

        return new SequentialAction(
                maxTimeFirstPath,
                maxTimeSecondPath,
                maxTimeThirdPath
        );
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
public Action goToRedGate() {
        TrajectoryActionBuilder redGateMove = drive.actionBuilder(redSecondLine)
                .setTangent(0)
                .splineTo(redGate.position, redGate.heading)
                .lineToY(35)
                .splineTo(redFarShootingPosition.position, redFarShootingPosition.heading);
        return redGateMove.build();
}

    //blue actions
    public Action blueMoveOffLine(boolean isClose) {
        TrajectoryActionBuilder moveOffLine = drive.actionBuilder(blueCloseShootingPosition)
                .setTangent(0)
                .splineToLinearHeading(blueEnd, Math.toRadians(-90));
        return moveOffLine.build();
    }

    public Action blueDriveCloseShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .lineToXLinearHeading(-35, Math.toRadians(-125));
        return moveOffWall.build();
    }

    public Action blueFirstLineShots(boolean isClose) {
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

    public Action blueSecondLineShots(boolean isClose) {
        Action firstLineShot = drive.actionBuilder(isClose ? blueCloseShootingPosition : blueFarShootingPosition)
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

    public Action blueThirdLineShots(boolean isClose) {
        Action firstLineShot = drive.actionBuilder(isClose ? blueCloseShootingPosition : blueFarShootingPosition)
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
