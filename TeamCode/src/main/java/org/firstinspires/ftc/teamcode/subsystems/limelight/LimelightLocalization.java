package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;

import java.util.ArrayList;
import java.util.List;

@Config
public class LimelightLocalization extends LLParent {
    public enum UpdatePoseType {
        CONTINUOUS,
        ON_COMMAND
    }
    public enum LocalizationState {
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
        public double minTimeBetweenUpdates = 5, limelightConnectionBufferTime = 0.5;
        public boolean useMT2 = false;
        public int numPrevPosesToPrint = 0;
        public LocalizationState offLocalizationState = LocalizationState.PASSIVE_READING;
    }

    public static UpdatePoseType updatePoseType = UpdatePoseType.CONTINUOUS;
    public static UpdatePoseParams updatePoseParams = new UpdatePoseParams();
    private Pose2d turretPose, robotPose;
    private Vector2d robotTurretVec;
    private LLResult aprilTagResult;
    private Pose2d lastAvgTurretPose;
    private final ArrayList<Pose3D> lastTurretPoses;
    public double maxTranslationalError, maxHeadingErrorDeg;
    private boolean drivetrainGoodForUpdate, turretGoodForUpdate;
    public boolean successfullyFoundPose;
    private LocalizationState state, prevState;
    private final ElapsedTime stateTimer;
    private int numSetPoses = 0;
    private double lastUpdatePoseTimeMs = 0;
    public boolean manualPoseUpdate;
    public List<LLResultTypes.FiducialResult> visibleTagInfo;

    public LimelightLocalization(BrainSTEMRobot robot, Limelight3A limelight) {
        super(robot, limelight);

        robotPose = new Pose2d(0, 0, 0);
        turretPose = new Pose2d(0, 0, 0);
        robotTurretVec = new Vector2d(0, 0);
        lastAvgTurretPose = new Pose2d(0, 0, 0);
        lastTurretPoses = new ArrayList<>();
        stateTimer = new ElapsedTime();
        prevState = LocalizationState.OFF;
        setState(LocalizationState.OFF);
        successfullyFoundPose = false;
        drivetrainGoodForUpdate = false;
        turretGoodForUpdate = false;
        manualPoseUpdate = false;
        maxTranslationalError = 0;
        maxHeadingErrorDeg = 0;
        numSetPoses = 0;
        visibleTagInfo = new ArrayList<>();
    }

    public LocalizationState getState() {
        return state;
    }
    public LocalizationState getPrevState() {
        return prevState;
    }
    public double getStateTime() {
        return stateTimer.seconds();
    }
    public void setState(LocalizationState state) {
        if (state == this.state)
            return;
        stateTimer.reset();
        prevState = this.state;
        this.state = state;
        if (state == LocalizationState.UPDATING_POSE) {
            lastTurretPoses.clear();
            successfullyFoundPose = false;
        }
    }
    public Pose2d getRobotPose() {
        if(robotPose == null)
            return null;
        return new Pose2d(robotPose.position.x, robotPose.position.y, robotPose.heading.toDouble());
    }

    public void update() {
        drivetrainGoodForUpdate = canUpdateDrivetrainReliably();
        turretGoodForUpdate = canUpdateTurretReliably();

        if (!drivetrainGoodForUpdate || !turretGoodForUpdate) {
            // want to update again immediately if current update is interrupted
            if (state == LocalizationState.UPDATING_POSE)
                lastUpdatePoseTimeMs = -1;

            setState(updatePoseParams.offLocalizationState);
            lastTurretPoses.clear();
        }

        // only want to update again if robot moves, not if turret moves
        if (!drivetrainGoodForUpdate)
            successfullyFoundPose = false;

        double curTimeMs = System.currentTimeMillis();
        double timeSinceUpdate = (curTimeMs - lastUpdatePoseTimeMs) * 0.001;

        boolean canUpdate = drivetrainGoodForUpdate && turretGoodForUpdate &&
                !successfullyFoundPose && updatePoseType == UpdatePoseType.CONTINUOUS;

        // if the code has reached this ine, everything is ready to update the pose
        if (canUpdate && timeSinceUpdate >= updatePoseParams.minTimeBetweenUpdates && state != LocalizationState.UPDATING_POSE) {
            manualPoseUpdate = false;
            setState(LocalizationState.UPDATING_POSE);
        }

        switch (state) {
            case OFF:
                break;
            case PASSIVE_READING:
                updatePoseFromCamera();
                if (robotPose == null || !aprilTagResult.isValid()) {
                    robotPose = new Pose2d(0, 0, 0);
                    break;
                }
                updateMaxErrors(lastAvgTurretPose, turretPose);

                break;
            case UPDATING_POSE:
                updatePoseFromCamera();

                successfullyFoundPose = robotPose != null;
                if (!successfullyFoundPose) {
                    robotPose = new Pose2d(0, 0, 0);
                    break;
                }
                updateMaxErrors(lastAvgTurretPose, turretPose);

                robot.drive.localizer.setPose(robotPose);
                numSetPoses++;
                lastUpdatePoseTimeMs = System.currentTimeMillis();
                setState(updatePoseParams.offLocalizationState);
                break;
        }
    }
    public void updateTelemetry(Telemetry telemetry) {
        if(aprilTagResult != null) {
            telemetry.addData("   localization state", state);
            telemetry.addData("   isValid", aprilTagResult.isValid());
            telemetry.addData("   bot pose is null", aprilTagResult.getBotpose() == null);
            telemetry.addData("   drivetrain good for update", drivetrainGoodForUpdate);
            telemetry.addData("   turret good for update", turretGoodForUpdate);
            telemetry.addData("   time since last update", (System.currentTimeMillis() - lastUpdatePoseTimeMs) * 0.001);
            telemetry.addData("   successfully found pose", successfullyFoundPose);
            telemetry.addLine();
            telemetry.addData("   num visible tags", visibleTagInfo.size());
            StringBuilder tagIDs = new StringBuilder();
            for (LLResultTypes.FiducialResult result : visibleTagInfo)
                tagIDs.append(result.getFiducialId()).append(" ");
            telemetry.addData("   tag IDs", tagIDs);
            telemetry.addData("   max translational error", maxTranslationalError);
            telemetry.addData("   max heading error", maxHeadingErrorDeg);

            telemetry.addData("turret pose", MathUtils.formatPose2(turretPose));
            telemetry.addData("robot pose", MathUtils.formatPose2(robotPose));

            for (int i=0; i<Math.min(updatePoseParams.numPrevPosesToPrint, lastTurretPoses.size()); i++)
                telemetry.addData("   last pose " + (i + 1),
                        MathUtils.format2(lastTurretPoses.get(i).getPosition().x) + " " +
                                MathUtils.format2(lastTurretPoses.get(i).getPosition().y) + " " +
                                MathUtils.format2(lastTurretPoses.get(i).getOrientation().getYaw(AngleUnit.DEGREES)));

            telemetry.addData("turret vel", robot.turret.turretMotor.getVelocity());
        }
        else
            telemetry.addLine("   result is null");
    }
    private void updatePoseFromCamera() {
        if (turretPose != null)
            lastAvgTurretPose = new Pose2d(turretPose.position, turretPose.heading);
        else
            lastAvgTurretPose = new Pose2d(0, 0, 0);

        if (updatePoseParams.useMT2) {
            double turretHeadingDeg = Math.toDegrees(robot.turret.currentAngleRad);
            limelight.updateRobotOrientation(turretHeadingDeg);
        }

        aprilTagResult = limelight.getLatestResult();
        if (aprilTagResult == null || !aprilTagResult.isValid())
            return;

        visibleTagInfo = aprilTagResult.getFiducialResults();
        Pose3D curFrameTurretPose = aprilTagResult.getBotpose();
        if (curFrameTurretPose == null) {
            turretPose = null;
            robotPose = null;
            return;
        }

        lastTurretPoses.add(new Pose3D(curFrameTurretPose.getPosition().toUnit(DistanceUnit.INCH), curFrameTurretPose.getOrientation()));
        if (lastTurretPoses.size() > updatePoseParams.numPrevFramesToAvg)
            lastTurretPoses.remove(0);
        else {
            turretPose = null;
            robotPose = null;
            return;
        }

        turretPose = getAvgTurretPose(lastTurretPoses);
        robotPose = calculateRobotPose(turretPose);
    }
    private void updateMaxErrors(Pose2d lastAvgTurretPose, Pose2d curAvgTurretPose) {
        if (lastAvgTurretPose == null || curAvgTurretPose == null)
            return;
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
    private boolean canUpdateDrivetrainReliably() {
        OdoInfo odoVel = robot.drive.pinpoint().getMostRecentVelocity();
        Pose2d odoPose = robot.drive.localizer.getPose();
        Pose2d targetPose = robot.turret.targetPose;
        boolean slowEnough = Math.abs(Math.toDegrees(odoVel.headingRad)) < updatePoseParams.maxUpdateHeadingDegVel && Math.hypot(odoVel.x, odoVel.y) < updatePoseParams.maxUpdateTranslationalVel;
        boolean inRange = (numSetPoses == 0 && updatePoseParams.allowUpdateAnywhereForFirst) ||
                Math.hypot(odoPose.position.x - targetPose.position.x, odoPose.position.y - targetPose.position.y) < updatePoseParams.maxUpdateDist;
        return slowEnough && inRange;
    }
    private boolean canUpdateTurretReliably() {
        return robot.turret.turretMotor.getVelocity() < updatePoseParams.maxUpdateTurretVelTicksPerSec;
    }
}
