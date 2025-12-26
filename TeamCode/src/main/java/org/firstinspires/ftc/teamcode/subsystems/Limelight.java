
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.opmode.Alliance;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Limelight extends Component {
    public enum UpdatePoseType {
        CONTINUOUS,
        ON_COMMAND
    }
    public enum Pipeline {
        APRIL_TAG,
        OBJECT_DETECTION
    }
    public enum UpdateState {
        OFF,
        PASSIVE_READING,
        UPDATING_POSE
    }
    public static class UpdatePoseParams {
        public double maxUpdateTranslationalVel = 2, maxUpdateHeadingDegVel = 2; // inches and degrees
        public int maxUpdateTurretVelTicksPerSec = 1;
        public boolean allowUpdateAnywhereForFirst = true;
        public double maxUpdateDist = 800;
        public int numPrevFramesToAvg = 5;
        public int minTimeBetweenUpdates = 5;
        public boolean useMT2 = false;
        public int numPrevPosesToPrint = 0;
    }
    public static class SnapshotParams {
        public String snapshotName = "near zone";
        public int snapshotNum = 0;
        public boolean clearSnapshots = false;
    }
    public static class ObjectDetectionParams {
        public boolean close = false;
    }
    public static UpdatePoseType updatePoseType = UpdatePoseType.CONTINUOUS;
    public static UpdateState offUpdateState = UpdateState.PASSIVE_READING;
    public static UpdatePoseParams updatePoseParams = new UpdatePoseParams();
    public static SnapshotParams snapshotParams = new SnapshotParams();
    public static ObjectDetectionParams objectDetectionParams = new ObjectDetectionParams();

    // i should tune the camera so that it gives me the turret center position
    public final Limelight3A limelight;
    private Pipeline pipeline;
    private Pose2d turretPose, robotPose;
    private Vector2d robotTurretVec;
    private LLResult aprilTagResult;
    private Pose2d lastAvgTurretPose;
    private final ArrayList<Pose3D> lastTurretPoses;
    public double maxTranslationalError, maxHeadingErrorDeg;
    private boolean drivetrainGoodForUpdate, turretGoodForUpdate;
    public boolean successfullyFoundPose;
    private UpdateState updateState, prevUpdateState;
    private final ElapsedTime stateTimer;
    private OdoInfo odoVel = new OdoInfo(0, 0, 0);
    private Pose2d odoPose = new Pose2d(0, 0, 0);
    private int numSetPoses = 0;
    private double lastUpdateTimeMs = 0;
    public boolean manualPoseUpdate;
    public List<LLResultTypes.FiducialResult> visibleTagInfo;
    public double[] objectDetectionOutput;
    public Limelight(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        robotPose = new Pose2d(0, 0, 0);
        turretPose = new Pose2d(0, 0, 0);
        robotTurretVec = new Vector2d(0, 0);
        lastAvgTurretPose = new Pose2d(0, 0, 0);
        lastTurretPoses = new ArrayList<>();
        stateTimer = new ElapsedTime();
        prevUpdateState = UpdateState.PASSIVE_READING;
        setState(UpdateState.PASSIVE_READING);
        successfullyFoundPose = false;
        drivetrainGoodForUpdate = false;
        turretGoodForUpdate = false;
        manualPoseUpdate = false;
        maxTranslationalError = 0;
        maxHeadingErrorDeg = 0;
        numSetPoses = 0;
        pipeline = Pipeline.APRIL_TAG;
        objectDetectionOutput = new double[0];
    }

    @Override
    public void printInfo() {
        telemetry.addLine("LIMELIGHT");
        telemetry.addData("state", updateState);
        telemetry.addData("   num set poses", numSetPoses);

        telemetry.addLine();
        switch (pipeline) {
            case APRIL_TAG:
                updateAprilTagTelemetry();
                break;
            case OBJECT_DETECTION:
                updateObjectDetectionTelemetry();
                break;
        }
    }
    private void updateAprilTagTelemetry() {
        if(aprilTagResult != null) {
            telemetry.addData("   isValid", aprilTagResult.isValid());
            telemetry.addData("   bot pose is null", aprilTagResult.getBotpose() == null);
            telemetry.addData("   drivetrain good for update", drivetrainGoodForUpdate);
            telemetry.addData("   turret good for update", turretGoodForUpdate);
            telemetry.addData("   successfully found pose", successfullyFoundPose);
            telemetry.addLine();
            telemetry.addData("   num visible tags", visibleTagInfo.size());
            StringBuilder tagIDs = new StringBuilder();
            for (LLResultTypes.FiducialResult result : visibleTagInfo)
                tagIDs.append(result.getFiducialId()).append(" ");
            telemetry.addData("   tag IDs", tagIDs);
            telemetry.addData("   max translational error", maxTranslationalError);
            telemetry.addData("   max heading error", maxHeadingErrorDeg);
            telemetry.addData("   turret pose", MathUtils.format2(turretPose.position.x) + " " + MathUtils.format2(turretPose.position.y) + " " + MathUtils.format2(Math.toDegrees(turretPose.heading.toDouble())));
            telemetry.addData("   robot pose", MathUtils.format2(robotPose.position.x) + " " + MathUtils.format2(robotPose.position.y) + " " + MathUtils.format2(Math.toDegrees(robotPose.heading.toDouble())));

            for (int i=0; i<Math.min(updatePoseParams.numPrevPosesToPrint, lastTurretPoses.size()); i++)
                telemetry.addData("   last pose " + (i + 1),
                        MathUtils.format2(lastTurretPoses.get(i).getPosition().x) + " " +
                                MathUtils.format2(lastTurretPoses.get(i).getPosition().y) + " " +
                                MathUtils.format2(lastTurretPoses.get(i).getOrientation().getYaw(AngleUnit.DEGREES)));

            telemetry.addData("odo vel", odoVel.toString(2));
            telemetry.addData("odo pose", MathUtils.format2Pose(odoPose));
            telemetry.addData("turret vel", robot.turret.turretMotor.getVelocity());
        }
        else
            telemetry.addLine("   result is null");
    }
    private void updateObjectDetectionTelemetry() {
        telemetry.addData("python outputs", Arrays.toString(objectDetectionOutput));
    }

    @Override
    public void update() {
        if (snapshotParams.clearSnapshots) {
            limelight.deleteSnapshots();
            snapshotParams.clearSnapshots = false;
        }

        switch (pipeline) {
            case APRIL_TAG:
                updateAprilTag();
                break;
            case OBJECT_DETECTION:
                updateObjectDetection();
                break;
        }
    }

    private void updateAprilTag() {
        drivetrainGoodForUpdate = canUpdateDrivetrainReliably();
        turretGoodForUpdate = canUpdateTurretReliably();

        if (!drivetrainGoodForUpdate || !turretGoodForUpdate) {
            // want to update again immediately if current update is interrupted
            if (updateState == UpdateState.UPDATING_POSE)
                lastUpdateTimeMs = 0;

            setState(offUpdateState);
            lastTurretPoses.clear();
        }

        if (!drivetrainGoodForUpdate)
            successfullyFoundPose = false;

        double curTimeMs = System.currentTimeMillis();
        if (drivetrainGoodForUpdate && turretGoodForUpdate && !successfullyFoundPose &&
                updatePoseType == UpdatePoseType.CONTINUOUS &&
                (curTimeMs - lastUpdateTimeMs) * 0.001 > updatePoseParams.minTimeBetweenUpdates) {
            lastUpdateTimeMs = curTimeMs;
            manualPoseUpdate = false;
            setState(UpdateState.UPDATING_POSE);
        }

        switch (updateState) {
            case OFF:
                break;
            case PASSIVE_READING:
                robotPose = updatePoseFromCamera();
                if (robotPose == null || !aprilTagResult.isValid()) {
                    robotPose = new Pose2d(0, 0, 0);
                    break;
                }
                updateMaxErrors(lastAvgTurretPose, turretPose);

                break;
            case UPDATING_POSE:
                if (!drivetrainGoodForUpdate) {
                    successfullyFoundPose = false;
                    setState(offUpdateState);
                    break;
                }
                robotPose = updatePoseFromCamera();
                if (!aprilTagResult.isValid()) {
                    robotPose = new Pose2d(0, 0, 0);
                    successfullyFoundPose = false;
                    setState(offUpdateState);
                    break;
                }

                successfullyFoundPose = robotPose != null;
                if (!successfullyFoundPose) {
                    robotPose = new Pose2d(0, 0, 0);
                    break;
                }
                updateMaxErrors(lastAvgTurretPose, turretPose);

                robot.drive.localizer.setPose(robotPose);
                numSetPoses++;
                setState(offUpdateState);
                break;
        }
    }
    private void updateObjectDetection() {
        double[] inputs = new double[2];
        inputs[0] = robot.alliance == Alliance.RED ? 1 : -1;
        inputs[1] = objectDetectionParams.close ? 1 : 0;

        limelight.updatePythonInputs(inputs);

        LLResult result = limelight.getLatestResult();
        objectDetectionOutput = result.getPythonOutput();
    }
    public void switchPipeline(Pipeline pipeline) {
        if (this.pipeline == pipeline)
            return;

        this.pipeline = pipeline;

        int pipelineIndex;
        switch (pipeline) {
            case OBJECT_DETECTION:
                pipelineIndex = 1;
                break;
            case APRIL_TAG:
            default:
                pipelineIndex = 0;
        }
        limelight.pipelineSwitch(pipelineIndex);
    }
    private Pose2d updatePoseFromCamera() {
        lastAvgTurretPose = new Pose2d(turretPose.position, turretPose.heading);

        if (updatePoseParams.useMT2) {
            double turretHeadingDeg = Math.toDegrees(robot.turret.currentAngleRad);
            limelight.updateRobotOrientation(turretHeadingDeg);
        }
        
        aprilTagResult = limelight.getLatestResult();
        if (aprilTagResult == null || !aprilTagResult.isValid())
            return null;

        visibleTagInfo = aprilTagResult.getFiducialResults();
        Pose3D curFrameTurretPose = aprilTagResult.getBotpose();
        if (curFrameTurretPose == null)
            return null;

        lastTurretPoses.add(new Pose3D(curFrameTurretPose.getPosition().toUnit(DistanceUnit.INCH), curFrameTurretPose.getOrientation()));
        if (lastTurretPoses.size() > updatePoseParams.numPrevFramesToAvg)
            lastTurretPoses.remove(0);
        else
            return null;

        turretPose = getAvgTurretPose(lastTurretPoses);
        return calculateRobotPose(turretPose);
    }
    private void updateMaxErrors(Pose2d lastAvgTurretPose, Pose2d curAvgTurretPose) {
        double translationalError = Math.hypot(curAvgTurretPose.position.x - lastAvgTurretPose.position.x, curAvgTurretPose.position.y - lastAvgTurretPose.position.y);
        double headingErrorDeg = Math.abs(Math.toDegrees(curAvgTurretPose.heading.toDouble() - lastAvgTurretPose.heading.toDouble()));
        maxTranslationalError = Math.max(maxTranslationalError, translationalError);
        maxHeadingErrorDeg = Math.max(maxHeadingErrorDeg, headingErrorDeg);
    }
    private Pose2d getAvgTurretPose(ArrayList<Pose3D> lastTurretPoses) {
        double x = 0, y = 0, hRad = 0;
        for (Pose3D pose : lastTurretPoses) {
            x += pose.getPosition().x;
            y += pose.getPosition().y;
            hRad += pose.getOrientation().getYaw(AngleUnit.RADIANS);
        }
        int num = lastTurretPoses.size();
        return new Pose2d(x / num, y / num, hRad / num);
    }
    private Pose2d calculateRobotPose(Pose2d turretPose) {
        int currentTurretPosition = robot.turret.turretMotor.getCurrentPosition();
        double relTurretAngleRad = Turret.getTurretRelativeAngleRad(currentTurretPosition);
        double robotHeading = turretPose.heading.toDouble() - relTurretAngleRad;
        if(robotHeading > Math.PI)
            robotHeading -= Math.PI * 2;
        robotTurretVec = new Vector2d(Turret.offsetFromCenter * Math.cos(robotHeading), Turret.offsetFromCenter * Math.sin(robotHeading));
        return new Pose2d(turretPose.position.x + robotTurretVec.x, turretPose.position.y + robotTurretVec.y, robotHeading);
    }
    public Pose2d getRobotPose() {
        if(robotPose == null)
            return null;
        return new Pose2d(robotPose.position.x, robotPose.position.y, robotPose.heading.toDouble());
    }
    private boolean canUpdateDrivetrainReliably() {
        odoVel = robot.drive.pinpoint().getMostRecentVelocity();
        odoPose = robot.drive.localizer.getPose();
        Pose2d targetPose = robot.turret.targetPose;
        boolean slowEnough = Math.abs(Math.toDegrees(odoVel.headingRad)) < updatePoseParams.maxUpdateHeadingDegVel && Math.hypot(odoVel.x, odoVel.y) < updatePoseParams.maxUpdateTranslationalVel;
        boolean inRange = (numSetPoses == 0 && updatePoseParams.allowUpdateAnywhereForFirst) ||
                Math.hypot(odoPose.position.x - targetPose.position.x, odoPose.position.y - targetPose.position.y) < updatePoseParams.maxUpdateDist;
        return slowEnough && inRange;
    }
    private boolean canUpdateTurretReliably() {
        return robot.turret.turretMotor.getVelocity() < updatePoseParams.maxUpdateTurretVelTicksPerSec;
    }

    public UpdateState getState() {
        return updateState;
    }
    public UpdateState getPrevState() {
        return prevUpdateState;
    }
    public double getStateTime() {
        return stateTimer.seconds();
    }
    public void setState(UpdateState state) {
        if (state == this.updateState)
            return;
        stateTimer.reset();
        prevUpdateState = updateState;
        updateState = state;
        if (state == UpdateState.UPDATING_POSE) {
            lastTurretPoses.clear();
            successfullyFoundPose = false;
        }
    }

    public void takePic() {
        limelight.captureSnapshot(snapshotParams.snapshotName + "-" + snapshotParams.snapshotNum + " | " + MathUtils.format2Pose(robotPose));
        snapshotParams.snapshotNum++;
    }
}

