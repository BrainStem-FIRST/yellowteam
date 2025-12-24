package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 128;
    public static double printPowerThreshold = 0.95;
    private boolean pathFinished;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-DISTANCE * 0.5, 0, 0));
            waitForStart();

            while (opModeIsActive()) {
                pathFinished = false;
                Action drivePath = drive.actionBuilder(new Pose2d(-DISTANCE * 0.5, 0, 0))
                        .lineToX(DISTANCE * 0.5)
                        .lineToX(-DISTANCE * 0.5)
                        .build();
                Actions.runBlocking(
                        new ParallelAction(
                                new SequentialAction(
                                        drivePath,
                                        packet -> {
                                            pathFinished = true;
                                            return false;
                                        }
                                ),

                                packet -> {
                                    telemetry.addData("FL drive power", drive.leftFront.getPower());
                                    telemetry.addData("drive powers", drive.getDrivePowersString());
                                    if (drive.leftFront.getPower() > printPowerThreshold)
                                        telemetry.addLine("drive power FL greater than " + printPowerThreshold);
                                    telemetry.update();
                                    return !pathFinished;
                                }
                        )
                    );
            }
        }
        else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
            }
        } else {
            throw new RuntimeException();
        }
    }
}
