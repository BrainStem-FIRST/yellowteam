package org.firstinspires.ftc.teamcode.opmode.testing;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Config
@TeleOp (name = "April Tag", group = "Robot")
public class AprilTagTest extends LinearOpMode {

    public static class Params{
        public int exposure = 20;
        public int gain = 10;
    }

    public static Params VISION_PARAMS = new AprilTagTest.Params();

    @Override
    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(new Position(DistanceUnit.INCH, 0, 0, 0, 0), new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 0, 0))
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(640, 480))
                .build();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(visionPortal, 0); //192.168.43.1:8080

        double time = System.currentTimeMillis();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            double prev = System.currentTimeMillis();
            if (!tagProcessor.getDetections().isEmpty()){
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("prev", prev);
                double cur = System.currentTimeMillis();
                telemetry.addData("cur", cur);
                telemetry.addData("fps", 1000/(cur - prev));
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("yaw", tag.ftcPose.yaw);
            }

            telemetry.update();

        }

        visionPortal.close();

    }
}