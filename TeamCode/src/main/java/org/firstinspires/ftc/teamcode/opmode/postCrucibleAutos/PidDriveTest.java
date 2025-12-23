package org.firstinspires.ftc.teamcode.opmode.postCrucibleAutos;

import static org.firstinspires.ftc.teamcode.utils.math.MathUtils.createPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.pidDrive.DrivePath;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Tolerance;
import org.firstinspires.ftc.teamcode.utils.pidDrive.Waypoint;

@Autonomous(name="PID Drive Test")
@Config
public class PidDriveTest extends OpMode {
    public static double[] p0 = new double[] { 0, 0, 0 };
    public static double[] p1 = new double[] { 48, 0, 0 };
    public static double[] tol1 = new double[] { 1, 1 };
    public static double slowDown1 = 1;
    public static double[] p2 = new double[] { 0, 0, 0 };
    public static double[] tol2 = new double[] { 2, 2 };
    private MecanumDrive drive;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(20);

        Pose2d startPose = createPose(p0);
        drive = new MecanumDrive(hardwareMap, startPose);
        telemetry.addLine("ready");
        telemetry.update();
    }

    @Override
    public void start() {
        Pose2d pose1 = createPose(p1);
        Pose2d pose2 = createPose(p2);

        Waypoint w1 = new Waypoint(pose1, new Tolerance(tol1));
        w1.params.slowDownPercent = slowDown1;
        Waypoint w2 = new Waypoint(pose2, new Tolerance(tol2));

        DrivePath d1 = new DrivePath(drive, telemetry, w1, w2);

        Actions.runBlocking(d1);
    }
    @Override
    public void loop() {
    }
}
