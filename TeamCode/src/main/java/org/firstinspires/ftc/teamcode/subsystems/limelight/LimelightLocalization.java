package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.canvas.Canvas;
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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsystems.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryHelper;

import java.util.ArrayList;
import java.util.List;

@Config
public class LimelightLocalization extends LLParent {
    public enum LocalizationType {
        CONTINUOUS,
        ON_COMMAND
    }
    public enum LocalizationState {
        OFF,
        PASSIVE_READING,
        UPDATING_POSE
    }

    public static class Params {
        public boolean showTurretPoses = false;
        public double[] nearZoneLocalizeCircle = { -36, 0, 48 };
        public double[] validLocalizeYRange = { -24, 24 };
        public double maxUpdateTranslationalVel = 2, maxUpdateHeadingDegVel = 2; // inches and degrees
        public int maxUpdateTurretVelTicksPerSec = 1, turretUpdateEncoderRange = 3;
        public boolean allowUpdateAnywhereForFirst = true;
        public double ableToUpdateConfirmationTime = 0.2;
        public int numPrevFramesToAvg = 5;
        public double minTimeBetweenUpdates = 5;
        public boolean useMT2 = false;
        public int numPrevPosesToPrint = 0;
        public LocalizationState offLocalizationState = LocalizationState.PASSIVE_READING;
    }

    public static LocalizationType localizationType = LocalizationType.ON_COMMAND;
    public static Params params = new Params();
    public Pose2d turretPose, robotPose, rawTurretPose, rawRobotPose;
    private Vector2d robotTurretVec;
    private LLResult aprilTagResult;
    private Pose2d lastAvgTurretPose;
    private final ArrayList<Pose3D> lastTurretPoses;
    public double maxTranslationalError, maxHeadingErrorDeg;
    private boolean drivetrainGoodForUpdate, turretGoodForUpdate, inLocalizationZone;
    public boolean successfullyFoundPose;
    private LocalizationState state, prevState;
    private final ElapsedTime stateTimer;
    private int numSetPoses = 0;
    private double lastUpdatePoseTimeMs = 0;
    public boolean manualPoseUpdate;
    public List<LLResultTypes.FiducialResult> visibleTagInfo;
    private final ElapsedTime ableToUpdateTimer;
    private int numFramesInState;

    public LimelightLocalization(BrainSTEMRobot robot, Limelight3A limelight) {
        super(robot, limelight);

        robotPose = null;
        turretPose = null;
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
        ableToUpdateTimer = new ElapsedTime();
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
        numFramesInState = 0;
        prevState = this.state;
        this.state = state;
        if (state == LocalizationState.UPDATING_POSE) {
            lastTurretPoses.clear();
            successfullyFoundPose = false;
        }
    }

    public void update() {
        numFramesInState++;
        drivetrainGoodForUpdate = canUpdateDrivetrainReliably();
        turretGoodForUpdate = canUpdateTurretReliably();
        inLocalizationZone = isInLocalizationZone();

        if (!drivetrainGoodForUpdate || !turretGoodForUpdate || !inLocalizationZone) {
            // want to update again immediately if current update is interrupted
            if (state == LocalizationState.UPDATING_POSE)
                lastUpdatePoseTimeMs = -1;

            setState(params.offLocalizationState);
            lastTurretPoses.clear();
            ableToUpdateTimer.reset();
        }

        // only want to update again if robot moves, not if turret moves
        if (!drivetrainGoodForUpdate)
            successfullyFoundPose = false;

        double curTimeMs = System.currentTimeMillis();
        double timeSinceUpdate = (curTimeMs - lastUpdatePoseTimeMs) * 0.001;

        boolean canUpdate = drivetrainGoodForUpdate && turretGoodForUpdate && inLocalizationZone &&
                ableToUpdateTimer.seconds() >= params.ableToUpdateConfirmationTime &&
                !successfullyFoundPose && localizationType == LocalizationType.CONTINUOUS;

        // if the code has reached this ine, everything is ready to update the pose
        if (canUpdate && timeSinceUpdate >= params.minTimeBetweenUpdates && state != LocalizationState.UPDATING_POSE) {
            manualPoseUpdate = false;
            setState(LocalizationState.UPDATING_POSE);
            lastUpdatePoseTimeMs = System.currentTimeMillis();
        }

        switch (state) {
            case OFF:
                break;
            case PASSIVE_READING:
                updatePoseFromCamera();
                if (robotPose == null || !aprilTagResult.isValid())
                    break;
                updateMaxErrors(lastAvgTurretPose, turretPose);

                break;
            case UPDATING_POSE:
                if (numFramesInState > params.numPrevFramesToAvg * 2) {
                    setState(params.offLocalizationState);
                    break;
                }
                updatePoseFromCamera();

                successfullyFoundPose = robotPose != null;
                if (!successfullyFoundPose)
                    break;

                updateMaxErrors(lastAvgTurretPose, turretPose);

                robot.drive.localizer.setPose(robotPose);
                numSetPoses++;
                setState(params.offLocalizationState);
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
            telemetry.addData("   in localization zone", inLocalizationZone);
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

            for (int i = 0; i<Math.min(params.numPrevPosesToPrint, lastTurretPoses.size()); i++)
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

        if (params.useMT2) {
            double turretHeadingDeg = Math.toDegrees(robot.turret.currentAngleRad);
            limelight.updateRobotOrientation(turretHeadingDeg);
        }

        aprilTagResult = limelight.getLatestResult();
        if (aprilTagResult == null || !aprilTagResult.isValid())
            return;

        visibleTagInfo.clear();
        visibleTagInfo = aprilTagResult.getFiducialResults();
        boolean validTags = false;
        for (LLResultTypes.FiducialResult tagResult : visibleTagInfo)
            if (tagResult.getFiducialId() == 20 || tagResult.getFiducialId() == 24) {
                validTags = true;
                break;
            }
        if(!validTags) {
            setAllPosesToNull();
            setState(params.offLocalizationState);
            lastTurretPoses.clear();
            return;
        }
        Pose3D curFrameTurretPose = aprilTagResult.getBotpose();
        if (curFrameTurretPose == null) {
            setAllPosesToNull();
            setState(params.offLocalizationState);
            lastTurretPoses.clear();
            return;
        }
        Position curFrameTurretPosition = curFrameTurretPose.getPosition().toUnit(DistanceUnit.INCH);
        if (Math.abs(curFrameTurretPosition.x) < 0.00001 &&
            Math.abs(curFrameTurretPosition.y) < 0.00001 &&
            Math.abs(curFrameTurretPose.getOrientation().getYaw(AngleUnit.DEGREES)) < 0.00001) {
            setAllPosesToNull();
            setState(params.offLocalizationState);
            lastTurretPoses.clear();
            return;
        }

        rawTurretPose = new Pose2d(curFrameTurretPosition.x, curFrameTurretPosition.y, curFrameTurretPose.getOrientation().getYaw(AngleUnit.RADIANS));
        rawRobotPose = calculateRobotPose(rawTurretPose);

        lastTurretPoses.add(new Pose3D(curFrameTurretPosition, curFrameTurretPose.getOrientation()));
        if (lastTurretPoses.size() > params.numPrevFramesToAvg)
            lastTurretPoses.remove(0);
        else {
            setAllPosesToNull();
            return;
        }

        turretPose = getAvgTurretPose(lastTurretPoses);
        robotPose = calculateRobotPose(turretPose);
    }
    private void setAllPosesToNull() {
        turretPose = null;
        robotPose = null;
        rawTurretPose = null;
        rawRobotPose = null;
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
        return Math.abs(Math.toDegrees(odoVel.headingRad)) < params.maxUpdateHeadingDegVel && Math.hypot(odoVel.x, odoVel.y) < params.maxUpdateTranslationalVel;
    }
    private boolean canUpdateTurretReliably() {
        return robot.turret.turretMotor.getVelocity() < params.maxUpdateTurretVelTicksPerSec && Math.abs(robot.turret.getTurretEncoder()) <= params.turretUpdateEncoderRange;
    }
    private boolean isInLocalizationZone() {
        Pose2d odoPose = robot.drive.localizer.getPose();
        if (params.allowUpdateAnywhereForFirst && numSetPoses == 0)
            return true;

        if (odoPose.position.y >= params.validLocalizeYRange[0] && odoPose.position.y <= params.validLocalizeYRange[1])
            return true;
        return Math.hypot(odoPose.position.x - params.nearZoneLocalizeCircle[0], odoPose.position.y -  params.nearZoneLocalizeCircle[1]) < params.nearZoneLocalizeCircle[2];
    }

    public void addLocalizationInfo(Canvas fieldOverlay) {
        fieldOverlay.setStroke("yellow");
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.strokeCircle(params.nearZoneLocalizeCircle[0], params.nearZoneLocalizeCircle[1], params.nearZoneLocalizeCircle[2]);
        fieldOverlay.strokeRect(-72, params.validLocalizeYRange[0], 144, params.validLocalizeYRange[1] - params.validLocalizeYRange[0]);
        fieldOverlay.setStrokeWidth(2);

        if (params.showTurretPoses) {
            Pose2d limelightTurretPose = turretPose == null ? new Pose2d(0, 0, 0) : new Pose2d(turretPose.position, turretPose.heading);
            Pose2d rawLimelightTurretPose = rawTurretPose == null ? new Pose2d(0, 0, 0) : new Pose2d(rawTurretPose.position, rawTurretPose.heading);
            TelemetryHelper.radii[0] = 5;
            TelemetryHelper.radii[1] = 5;
            TelemetryHelper.colors[0] = "black";
            TelemetryHelper.colors[1] = "gray";
            TelemetryHelper.numPosesToShow = 2;
            TelemetryHelper.addRobotPoseToCanvas(fieldOverlay, limelightTurretPose, rawLimelightTurretPose);
        }
        else {
            TelemetryHelper.colors[0] = "gray";
            TelemetryHelper.numPosesToShow = 1;
            Pose2d robotPoseToDraw = robotPose == null ? new Pose2d(0, 0, 0) : new Pose2d(robotPose.position, robotPose.heading);
            TelemetryHelper.addRobotPoseToCanvas(fieldOverlay, robotPoseToDraw);
        }
    }
}
