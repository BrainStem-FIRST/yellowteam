package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class AutoPositions {
    private final MecanumDrive drive;

    //Red positions
    public final Pose2d redFarShootingPosition = new Pose2d(50, 10, Math.toRadians(180));
    public final Pose2d redCloseShootingPosition = new Pose2d(-40, 40, Math.toRadians(55));
    public final Pose2d redFirstLine = new Pose2d(-13, 26, Math.toRadians(90));
    public final Pose2d redSecondLine = new Pose2d(12, 26, Math.toRadians(90));
    public final Pose2d redThirdLine = new Pose2d(32, 26, Math.toRadians(90));
    public final Pose2d redApproachHP = new Pose2d(35, 68, Math.toRadians(45));
    public final Pose2d redEnd = new Pose2d(0, 45, Math.toRadians(90));

    //blue positions
    public final Pose2d blueFarShootingPosition = new Pose2d(50, -10, Math.toRadians(180));
    public final Pose2d blueCloseShootingPosition = new Pose2d(-40, -40, Math.toRadians(-135));
    public final Pose2d blueFirstLine = new Pose2d(-13, -26, Math.toRadians(-90));
    public final Pose2d blueSecondLine = new Pose2d(12, -26, Math.toRadians(-90));
    public final Pose2d blueThirdLine = new Pose2d(32.75, -26, Math.toRadians(-90));
    public final Pose2d blueApproachHP = new Pose2d(35, -68, Math.toRadians(-135));
    public final Pose2d blueEnd = new Pose2d(0, -45, Math.toRadians(-90));

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
        TrajectoryActionBuilder secondPath = drive.actionBuilder(redFirstLine)
                .lineToY(60)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(isClose ? redCloseShootingPosition : redFarShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));

        Action maxTimeFirstPath = new TimedAction(firstLineShot, 5);

        return new SequentialAction(
                maxTimeFirstPath,
                secondPath.build()
        );
    }

    public Action secondLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(isClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redSecondLine, Math.toRadians(90))
                .lineToY(63)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(isClose ? redCloseShootingPosition : redFarShootingPosition, isClose ? Math.toRadians(135) : Math.toRadians(0));
        return firstLineShot.build();
    }

    public Action thirdLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(redFarShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redThirdLine, Math.toRadians(90))
                .lineToY(63)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(isClose ? redCloseShootingPosition : redFarShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));
        return firstLineShot.build();
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
    public Action blueMoveOffLine(boolean isClose) {
        TrajectoryActionBuilder moveOffLine = drive.actionBuilder(blueCloseShootingPosition)
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
        TrajectoryActionBuilder secondPath = drive.actionBuilder(blueFirstLine)
                .lineToY(-60)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(isClose ? blueCloseShootingPosition : blueFarShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));

        Action maxTimeFirstPath = new TimedAction(firstLineShot, 5);

        return new SequentialAction(
                maxTimeFirstPath,
                secondPath.build()
        );
    }

    public Action blueSecondLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(isClose ? blueCloseShootingPosition : blueFarShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(blueSecondLine, Math.toRadians(-90))
                .lineToY(-63)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(isClose ? blueCloseShootingPosition : blueFarShootingPosition, isClose ? Math.toRadians(-45) : Math.toRadians(0));
        return firstLineShot.build();
    }

    public Action blueThirdLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(blueCloseShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(blueThirdLine, Math.toRadians(-90))
                .lineToY(-63)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(isClose ? blueCloseShootingPosition : blueFarShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));
        return firstLineShot.build();
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
