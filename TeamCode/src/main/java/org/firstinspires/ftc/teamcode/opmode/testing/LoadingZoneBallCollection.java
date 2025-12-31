package org.firstinspires.ftc.teamcode.opmode.testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightBallDetection;
import org.firstinspires.ftc.teamcode.utils.autoHelpers.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

@TeleOp(name="Loading Zone Ball Collection")
public class LoadingZoneBallCollection extends OpMode {
    private BrainSTEMRobot robot;
    private AutoCommands autoCommands;
    private boolean runningAction = false;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);
        robot = new BrainSTEMRobot(Alliance.RED, telemetry, hardwareMap, new Pose2d(0, 0, 0));
        robot.limelight.switchPipeline(2);
        autoCommands = new AutoCommands(robot, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            runningAction = true;
            robot.drive.stop();

            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    strafeToBall(),
                                    autoCommands.runIntake(),
                                    driveForwards(),
                                    autoCommands.stopIntake(),
                                    telemetryPacket -> {runningAction = false; return false;}
                            ),
                            telemetryPacket -> {
                                autoCommands.updateRobot.run(new TelemetryPacket());
                                telemetry.update();
                                return runningAction;
                            }
                    )
            );
        }

        robot.update(false);

        robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        telemetry.addData("successful", robot.limelight.ballDetection.isSuccessful());
        telemetry.addData("tx", robot.limelight.ballDetection.getTargetX());
        telemetry.addData("time running", getRuntime());
        robot.limelight.printInfo();
        telemetry.update();

    }

    private Action strafeToBall() {
        return new Action() {
            private int numTimesUnsuccessful = 0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!robot.limelight.ballDetection.isSuccessful())
                    numTimesUnsuccessful++;
                if (numTimesUnsuccessful > 4 || robot.drive.localizer.getPose().position.x >= 71.5 - BrainSTEMRobot.width * 0.5) {
                    robot.drive.stop();
                    return false;
                }

                double strafePower = robot.limelight.ballDetection.getStrafePower();
                robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -strafePower), 0));

                return robot.limelight.ballDetection.getTargetX() > LimelightBallDetection.params.txErrorThreshold;
            }
        };
    }
    private Action driveForwards() {
        Pose2d pose = new Pose2d(64, robot.drive.localizer.getPose().position.y, Math.toRadians(90));
        Tolerance tolerance = new Tolerance(2, 3);
        Waypoint waypoint = new Waypoint(pose, tolerance);
        waypoint.params.maxLinearPower = 0.5;
        return new DrivePath(robot.drive, telemetry, waypoint);
    }
}
