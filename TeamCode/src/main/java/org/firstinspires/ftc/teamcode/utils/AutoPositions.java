package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class AutoPositions {
    private final MecanumDrive drive;

    public final Pose2d farShootingPosition = new Pose2d(50, 10, Math.toRadians(180));
    public final Pose2d closeShootingPosition = new Pose2d(-35, 24, Math.toRadians(135));
    public final Pose2d firstLine = new Pose2d(-12, 17.5, Math.toRadians(90));
    public final Pose2d secondLine = new Pose2d(12, 17.5, Math.toRadians(90));
    public final Pose2d thirdLine = new Pose2d(27, 20, Math.toRadians(90));
    public final Pose2d approachHP = new Pose2d(35, 68, Math.toRadians(45));

    public AutoPositions(MecanumDrive drive) {
        this.drive = drive;
    }

    public Action driveToFarShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .splineToConstantHeading(farShootingPosition.position, farShootingPosition.heading);
        return moveOffWall.build();
    }

    public Action driveCloseShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .splineToLinearHeading(closeShootingPosition, Math.toRadians(180));
        return moveOffWall.build();
    }

    public Action firstLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(isClose ? closeShootingPosition : farShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(firstLine, Math.toRadians(90))
                .waitSeconds(0.25)
                .lineToY(55)
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(isClose ? closeShootingPosition : farShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));
        return firstLineShot.build();
    }

    public Action secondLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(isClose ? closeShootingPosition : farShootingPosition)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(secondLine, Math.toRadians(90))
                .waitSeconds(0.25)
                .lineToY(63)
                .waitSeconds(0.5)
                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(isClose ? closeShootingPosition : farShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));
        return firstLineShot.build();
    }

    public Action thirdLineShots(boolean isClose) {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(farShootingPosition)
                .splineTo(thirdLine.position, thirdLine.heading)
                .waitSeconds(0.25)
                .lineToY(63)
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(isClose ? closeShootingPosition : farShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));
        return firstLineShot.build();
    }

    public Action humanPlayerShots(boolean isClose) {
        TrajectoryActionBuilder hpShot = drive.actionBuilder(isClose ? closeShootingPosition : farShootingPosition)
                .splineTo(approachHP.position, approachHP.heading)
                .waitSeconds(0.5)
                .lineToX(65)
                .waitSeconds(0.5)
                .setTangent(0)
                .splineToLinearHeading(isClose ? closeShootingPosition : farShootingPosition, isClose ? Math.toRadians(180) : Math.toRadians(0));
        return hpShot.build();
    }
}
