package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class AutoPositions {
    private final MecanumDrive drive;

    public final Pose2d shootingPosition = new Pose2d(50, 10, Math.toRadians(180));
    public final Pose2d firstLine = new Pose2d(31, 36, Math.toRadians(90));
    public final Pose2d approachHP = new Pose2d(45, 57, Math.toRadians(45));
    public final Pose2d atHP = new Pose2d(58, 64, Math.toRadians(45));

    public AutoPositions(MecanumDrive drive) {
        this.drive = drive;
    }

    public Action driveToShootingPose(Pose2d startPose) {
        TrajectoryActionBuilder moveOffWall = drive.actionBuilder(startPose)
                .splineToConstantHeading(shootingPosition.position, shootingPosition.heading);
        return moveOffWall.build();
    }

    public Action firstLineShots() {
        TrajectoryActionBuilder firstLineShot = drive.actionBuilder(shootingPosition)
                .splineTo(firstLine.position, firstLine.heading)
                .lineToY(66)
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(shootingPosition, Math.toRadians(0));
        return firstLineShot.build();
    }

    public Action humanPlayerShots() {
        TrajectoryActionBuilder hpShot = drive.actionBuilder(shootingPosition)
                .splineTo(approachHP.position, approachHP.heading)
                .waitSeconds(2)
                .splineToLinearHeading(atHP, Math.toRadians(0));
//                .waitSeconds(2)
//                .setTangent(0)
//                .splineTo(shootingPosition.position, shootingPosition.heading);
        return hpShot.build();
    }
}
