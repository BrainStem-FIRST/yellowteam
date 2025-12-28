
package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Component;

@Config
public class Limelight extends Component {
    public enum Pipeline {
        OFF,
        APRIL_TAG,
        OBJECT_DETECTION,
        CLASSIFIER_DETECTION,
    }
    public static class SnapshotParams {
        public String snapshotName = "near zone";
        public int snapshotNum = 0;
        public boolean clearSnapshots = false;
    }
    public static SnapshotParams snapshotParams = new SnapshotParams();

    // i should tune the camera so that it gives me the turret center position
    public final Limelight3A limelight;
    public static Pipeline pipeline = Pipeline.OFF;
    public final LimelightLocalization localization;
    public final LimelightClassifier classifier;

    // classifier detection data
    public Limelight(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        localization = new LimelightLocalization(robot, limelight);
        classifier = new LimelightClassifier(robot, limelight);
    }

    @Override
    public void printInfo() {
        telemetry.addLine("LIMELIGHT");
        telemetry.addData("pipeline type", pipeline);
        telemetry.addData("limelight is running", limelight.isRunning());
        telemetry.addData("limelight is connected", limelight.isConnected());

        telemetry.addLine();
        switch (pipeline) {
            case OFF:
                break;
            case APRIL_TAG:
                localization.updateTelemetry(telemetry);
                break;
            case CLASSIFIER_DETECTION:
                classifier.updateTelemetry(telemetry);
                break;
            case OBJECT_DETECTION:
                break;
        }
    }
    @Override
    public void update() {
        if (snapshotParams.clearSnapshots) {
            limelight.deleteSnapshots();
            snapshotParams.clearSnapshots = false;
        }

        switch (pipeline) {
            case OFF:
                break;
            case APRIL_TAG:
                localization.update();
                break;
            case CLASSIFIER_DETECTION:
                classifier.update();
                //if (classifier.getMostCommonNumBalls() != -1)
                 //   switchPipeline(Pipeline.OFF);
                break;
            case OBJECT_DETECTION:
                break;
        }
    }

    public void switchPipeline(Pipeline pipeline) {
        if (Limelight.pipeline == pipeline)
            return;

        Limelight.pipeline = pipeline;

        int pipelineIndex = -1;
        switch (pipeline) {
            case APRIL_TAG:
                pipelineIndex = 0;
                break;
            case CLASSIFIER_DETECTION:
                pipelineIndex = 1;
                classifier.resetForNewRead();
                break;
            case OBJECT_DETECTION:
                pipelineIndex = 2;
                break;
        }
        if (pipelineIndex == -1)
            limelight.stop();
        else {
            if (!limelight.isRunning())
                limelight.start();
            limelight.pipelineSwitch(pipelineIndex);
        }
    }

    public void takePic() {
        limelight.captureSnapshot(snapshotParams.snapshotName + "-" + snapshotParams.snapshotNum);
        snapshotParams.snapshotNum++;
    }
}