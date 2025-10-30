package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class AutoPositions {
    private final MecanumDrive drive;

    //Red positions
    public final Pose2d redFarShootingPosition = new Pose2d(50, 10, Math.toRadians(180));
    public final Pose2d redCloseShootingPosition = new Pose2d(-35, 24, Math.toRadians(135));
    public final Pose2d redFirstLine = new Pose2d(-12, 17.5, Math.toRadians(90));
    public final Pose2d redSecondLine = new Pose2d(12, 17.5, Math.toRadians(90));
    public final Pose2d redThirdLine = new Pose2d(27, 20, Math.toRadians(90));
    public final Pose2d redApproachHP = new Pose2d(35, 68, Math.toRadians(45));

   //blue positions
    public final Pose2d blueFarShootingPosition = new Pose2d(50, -10, Math.toRadians(180));
    public final Pose2d blueCloseShootingPosition = new Pose2d(-35, -24, Math.toRadians(45));
    public final Pose2d blueFirstLine = new Pose2d(-12, -17.5, Math.toRadians(-90));
    public final Pose2d blueSecondLine = new Pose2d(12, -17.5, Math.toRadians(-90));
    public final Pose2d blueThirdLine = new Pose2d(27, -20, Math.toRadians(-90));
    public final Pose2d blueApproachHP = new Pose2d(35, -68, Math.toRadians(-135));

    public AutoPositions(MecanumDrive drive) {
        this.drive = drive;
    }

    public Action driveToFarShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .splineToConstantHeading(redFarShootingPosition.position, redFarShootingPosition.heading);
        return moveOffWall.build();
    }

    public Action driveCloseShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .splineToLinearHeading(redCloseShootingPosition, Math.toRadians(180));
        return moveOffWall.build();
    }

    public Action firstLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(isClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redFirstLine, Math.toRadians(90))
//                .waitSeconds(0.1)
                .lineToY(42)
//                .waitSeconds(0.1)
                .setReversed(true)
                .splineToLinearHeading(isClose ? redCloseShootingPosition : redFarShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));
        return firstLineShot.build();
    }

    public Action secondLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(isClose ? redCloseShootingPosition : redFarShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(redSecondLine, Math.toRadians(90))
//                .waitSeconds(0.1)
                .lineToY(47)
//                .waitSeconds(0.1)
//                .setReversed(true)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(isClose ? redCloseShootingPosition : redFarShootingPosition, isClose ? Math.toRadians(135) : Math.toRadians(0));
        return firstLineShot.build();
    }

    public Action thirdLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(redFarShootingPosition)
                .splineTo(redThirdLine.position, redThirdLine.heading)
                .waitSeconds(0.25)
                .lineToY(63)
                .waitSeconds(0.5)
                .setReversed(true)
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
    public Action blueDriveToFarShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .splineToConstantHeading(blueFarShootingPosition.position, blueFarShootingPosition.heading);
        return moveOffWall.build();
    }

    public Action blueDriveCloseShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .splineToLinearHeading(blueCloseShootingPosition, Math.toRadians(180));
        return moveOffWall.build();
    }

    public Action blueFirstLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(isClose ? blueCloseShootingPosition : blueFarShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(blueFirstLine, Math.toRadians(-90))
//                .waitSeconds(0.1)
                .lineToY(-42)
//                .waitSeconds(0.1)
                .setReversed(true)
                .splineToLinearHeading(isClose ? blueCloseShootingPosition : blueFarShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));
        return firstLineShot.build();
    }

    public Action blueSecondLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(isClose ? blueCloseShootingPosition : blueFarShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(blueSecondLine, Math.toRadians(-90))
//                .waitSeconds(0.1)
                .lineToY(-47)
//                .waitSeconds(0.1)
//                .setReversed(true)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(isClose ? blueCloseShootingPosition : blueFarShootingPosition, isClose ? Math.toRadians(45) : Math.toRadians(0));
        return firstLineShot.build();
    }

    public Action blueThirdLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(blueFarShootingPosition)
                .splineTo(blueThirdLine.position, blueThirdLine.heading)
                .waitSeconds(0.25)
                .lineToY(-63)
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(isClose ? blueCloseShootingPosition : blueFarShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));
        return firstLineShot.build();
    }

    public Action blueHumanPlayerShots(boolean isClose) {
        TrajectoryActionBuilder hpShot = drive.actionBuilder(isClose ? blueCloseShootingPosition : blueFarShootingPosition)
                .splineTo(blueApproachHP.position, blueApproachHP.heading)
                .waitSeconds(0.5)
                .lineToX(-65)
                .waitSeconds(0.5)
                .setTangent(0)
                .splineToLinearHeading(isClose ? blueCloseShootingPosition : blueFarShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));
        return hpShot.build();
    }
}
