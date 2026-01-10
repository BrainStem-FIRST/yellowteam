package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;

@Config
@TeleOp(name="localization test")
public class LocalizationTest extends LinearOpMode {
    public static double startX = -8.8, startY = -7.3, startA = Math.PI;
    public static boolean drawRobotPoses = false, drawTurretPoses = true, drawCameraPose = true;
    public static boolean useMegaTag2 = false;
    private Limelight3A limelight3A;
    private Turret turret;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, startA));

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
        turret = new Turret(hardwareMap, telemetry, null);
        turret.resetEncoders();

        double maxXError = 0;
        double maxYError = 0;
        double maxTranslationalError = 0;
        double maxHeadingErrorRad = 0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                maxXError = 0;
                maxYError = 0;
                maxTranslationalError = 0;
                maxHeadingErrorRad = 0;
            }
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));


            drive.updatePoseEstimate();
            Pose2d pinpointRobotPose = drive.localizer.getPose();
            int turretEncoder = turret.getTurretEncoder();
            Pose2d pinpointTurretPose = Turret.getTurretPose(pinpointRobotPose, turretEncoder);

            if (useMegaTag2)
                limelight3A.updateRobotOrientation(Math.toDegrees(pinpointTurretPose.heading.toDouble()));
            Pose2d[] limelightPoses = getLimelightPoses(limelight3A.getLatestResult(), turretEncoder);

            double xError = pinpointRobotPose.position.x - limelightPoses[0].position.x;
            double yError = pinpointRobotPose.position.y - limelightPoses[0].position.y;
            double translError = Math.hypot(xError, yError);
            double headingErrorRad = Math.abs(pinpointRobotPose.heading.toDouble() - limelightPoses[0].heading.toDouble());

            maxXError = Math.max(maxXError, xError);
            maxYError = Math.max(maxYError, yError);
            maxTranslationalError = Math.max(maxTranslationalError, translError);
            maxHeadingErrorRad = Math.max(maxHeadingErrorRad, headingErrorRad);


            telemetry.addData("reset errors", "gamepad1.a");
            telemetry.addLine("Pinpoint======================");
            telemetry.addData("pinpoint robot pose", MathUtils.formatPose3(pinpointRobotPose));
            telemetry.addData("pinpoint turret pose", MathUtils.formatPose3(pinpointTurretPose));
            telemetry.addLine();

            telemetry.addLine("Limelight======================");
            telemetry.addData("ll robot pose", MathUtils.formatPose3(limelightPoses[0]));
            telemetry.addData("ll turret pose", MathUtils.formatPose3(limelightPoses[1]));
            telemetry.addData("ll camera pose", MathUtils.formatPose3(limelightPoses[2]));
            telemetry.addLine();

            telemetry.addLine("Errors==========================");
            telemetry.addData("max x error", MathUtils.format3(maxXError));
            telemetry.addData("max y error", MathUtils.format3(maxYError));
            telemetry.addData("max translational error", MathUtils.format3(maxTranslationalError));
            telemetry.addData("max heading error deg", MathUtils.format3(Math.toDegrees(maxHeadingErrorRad)));

            telemetry.update();

            if (drawRobotPoses || drawTurretPoses) {
                TelemetryPacket packet = new TelemetryPacket();
                if (drawRobotPoses) {
                    packet.fieldOverlay().setStroke("green");
                    Drawing.drawRobot(packet.fieldOverlay(), pinpointRobotPose);
                    packet.fieldOverlay().setStroke("gray");
                    Drawing.drawRobot(packet.fieldOverlay(), limelightPoses[0]);
                }
                if (drawTurretPoses) {
                    packet.fieldOverlay().setStroke("green");
                    Drawing.drawRobotSimple(packet.fieldOverlay(), pinpointTurretPose, 3);
                    packet.fieldOverlay().setStroke("gray");
                    Drawing.drawRobotSimple(packet.fieldOverlay(), limelightPoses[1], 3);
                }
                if (drawCameraPose) {
                    packet.fieldOverlay().setStroke("black");
                    Drawing.drawRobotSimple(packet.fieldOverlay(), limelightPoses[2], 1);
                }
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        }
    }

    private Pose2d[] getLimelightPoses(LLResult result, int turretEncoder) {
        Pose2d[] poses = new Pose2d[3];
        poses[0] = new Pose2d(0, 0, 0);
        poses[1] = new Pose2d(0, 0, 0);
        poses[2] = new Pose2d(0, 0, 0);

        if (result == null || !result.isValid())
            return poses;

        Pose3D cameraPose3D = result.getBotpose();

        Position cameraPos = cameraPose3D.getPosition().toUnit(DistanceUnit.INCH);
        double cameraHeading = cameraPose3D.getOrientation().getYaw(AngleUnit.RADIANS);
        Pose2d cameraPose = new Pose2d(cameraPos.x, cameraPos.y, cameraHeading);
        if (cameraPose.position.x == 0 && cameraPose.position.y == 0 && cameraPose.heading.toDouble() == 0)
            return poses;

        Pose2d turretPose = Limelight.getTurretPose(cameraPose);

        poses[0] = Turret.getRobotPose(turretPose, turretEncoder);
        poses[1] = turretPose;
        poses[2] = cameraPose;
        return poses;
    }
}
