package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import java.util.List;

public class Vision implements Component {
    public VisionPortal visionPortal;
    public AprilTagProcessor tagProcessor;
    private AprilTagDetection currentTag;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
//        tagProcessor = new AprilTagProcessor.Builder()
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .build();
//
//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(tagProcessor)
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
//                .setCameraResolution(new Size(640, 480))
//                .build();

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        dashboard.startCameraStream(visionPortal, 0); //192.168.43.1:8080
    }

    public boolean isTagVisible() {
        return currentTag != null;
    }

    public AprilTagDetection getCurrentTag() {
        return currentTag;
    }

    public boolean isCorrectTag(int tagId) {
        return currentTag != null && currentTag.id == tagId;
    }

    @Override
    public void reset() {

    }

    @Override
    public void update() {
//        List<AprilTagDetection> detections = tagProcessor.getDetections();
//        if (!detections.isEmpty()) {
//            currentTag = detections.get(0);
//        } else {
//            currentTag = null;
//        }
    }

    @Override
    public String test() {
        return "";
    }
}