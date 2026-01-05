
package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Component;

@Config
public class Limelight extends Component {
    public static class SnapshotParams {
        public String snapshotName = "near zone";
        public int snapshotNum = 0;
        public boolean clearSnapshots = false;
    }
    public static SnapshotParams snapshotParams = new SnapshotParams();

    // i should tune the camera so that it gives me the turret center position
    public final Limelight3A limelight;
    public static int startingPipeline = 0;
    private int pipeline;
    public final LimelightLocalization localization; // april tag localization
    public final LimelightClassifier classifier; // counts # balls in classifier
    public final LimelightBallDetection ballDetection; // detects balls in loading zone

    // classifier detection data
    public Limelight(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        localization = new LimelightLocalization(robot, limelight);
        classifier = new LimelightClassifier(robot, limelight);
        ballDetection = new LimelightBallDetection(robot, limelight);

        pipeline = -1;
        switchPipeline(startingPipeline);
        limelight.start();
    }

    @Override
    public void printInfo() {
        telemetry.addLine("LIMELIGHT");
        telemetry.addData("pipeline type", pipeline);
        telemetry.addData("limelight is running", limelight.isRunning());
        telemetry.addData("limelight is connected", limelight.isConnected());

        telemetry.addLine();
        switch (pipeline) {
            case 0:
                localization.updateTelemetry(telemetry);
                break;
            case 1:
                classifier.updateTelemetry(telemetry);
                break;
            case 2:
                ballDetection.updateTelemetry(telemetry);
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
            case 0:
                localization.update();
                break;
            case 1:
                classifier.update();
                break;
            case 2:
                ballDetection.update();
                break;
        }
    }

    public void switchPipeline(int num) {
        if (pipeline == num)
            return;

        pipeline = num;
        if (num == -1)
            limelight.stop();
        else if (!limelight.isRunning())
            limelight.start();

        limelight.pipelineSwitch(num);
    }

    public void takePic() {
        limelight.captureSnapshot(snapshotParams.snapshotName + "-" + snapshotParams.snapshotNum);
        snapshotParams.snapshotNum++;
    }

    public void addLimelightInfo(Canvas fieldOverlay) {
        switch (pipeline) {
            case 0:
                localization.addLocalizationInfo(fieldOverlay);
                break;
            case 1:
                classifier.addClassifierInfo(fieldOverlay);
                break;
            case 2:
                break;
        }
    }
}