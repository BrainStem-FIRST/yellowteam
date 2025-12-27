
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
        Pose2d turretPose = Turret.getTurretPose(robot.drive.localizer.getPose(), robot.turret.getTurretEncoder());

        double[] inputs = new double[3];
        inputs[0] = robot.alliance == Alliance.RED ? 1 : -1;
        inputs[1] = objectDetectionParams.close ? 1 : 0;
        inputs[2] = turretPose.position.y;

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
/*
import cv2
import numpy as np
import math

# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # mask image based on classifier
    valid_region_mask = get_classifier_mask(img_hsv, (90,130,100), (130,255,210))
    image = cv2.bitwise_and(image, image, mask=valid_region_mask)

    image = cv2.GaussianBlur(image, (3, 3), 0)
    # convert the input image to the HSV color space
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


    green_contours = get_contours(img_hsv, (50, 35, 90), (80, 255, 255))
    purple_contours = get_contours(img_hsv, (153, 35, 90), (170, 255, 240))

    # combine and sort lists
    sorted_contours = []
    sorted_contours.extend(green_contours)
    sorted_contours.extend(purple_contours)
    sorted_contours.sort(key=centroid_x)

    angle_rad = get_avg_angle_rad_between_sorted_contours(sorted_contours)
    rounded_angle = int(100 * angle_rad * 180 / math.pi) / 100
    cv2.putText(image, f"{rounded_angle}", (60, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0, (0, 0, 0), 2, cv2.LINE_AA)

    pivot = len(image[0]) * 0.5, len(image) * 0.5
    rotated_contours = rotate_contours(pivot, angle_rad, sorted_contours)

    rects = segment_contours(rotated_contours)

    rects = rotate_rects(pivot, angle_rad, rects)

    draw_combined_contours(image, sorted_contours)
    draw_rects(image, rects)

    show_sample_pixel(image, img_hsv, 600, 400)
    #return the largest contour for the LL crosshair, the modified image, and custom robot data

    return contour_at_point(600, 400), image, []

def get_classifier_mask(image, classifier_lower_bounds, classifier_upper_bounds):
    classifier_mask = cv2.inRange(image, classifier_lower_bounds, classifier_upper_bounds)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    classifier_mask = cv2.morphologyEx(classifier_mask, cv2.MORPH_OPEN, kernel)
    classifier_mask = cv2.morphologyEx(classifier_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(classifier_mask,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    classifier_mask = cv2.cvtColor(classifier_mask, cv2.COLOR_GRAY2BGR)

    mask = np.zeros((len(image), len(image[0])), dtype=np.uint8)
    if (len(contours) > 0):
        largest_contour = max(contours, key=cv2.contourArea)

        perimeter = cv2.arcLength(largest_contour, True)
        epsilon = 0.02 * perimeter
        approx = cv2.approxPolyDP(largest_contour, epsilon, closed=True)
        # cv2.polylines(classifier_mask, [approx],
        # isClosed=True, color=(0, 255, 0), thickness=2)

        length, edge1, length2, edge2 = longest_2edges_from_approx(approx)
        p1 = edge1[0]
        p2 = edge1[1]

        if (edge2 is not None):
            pp1 = edge2[0]
            pp2 = edge2[1]
            avgy1 = (p1[1] + p2[1]) * 0.5
            avgy2 = (pp1[1] + pp2[1]) * 0.5

            width1 = abs(p2[0] - p1[0])
            width2 = abs(pp2[0] - pp1[0])

            min_width_to_switch = 400

            if (avgy2 < avgy1 and width2 > min_width_to_switch):
                p1 = pp1
                p2 = pp2

        # generating rectangle above classifier
        r1x = p1[0]
        r1y = p1[1]
        r2x = p2[0]
        r2y = p2[1]
        if (p1[0] > p2[0]):
            r1x = p2[0]
            r1y = p2[1]
            r2x = p1[0]
            r2y = p1[1]

        dx = r2x - r1x
        dy = r2y - r1y

        r1x -= dx * 0.1
        r1y -= dy * 0.1
        r2x += dx * 0.1
        r2y += dy * 0.1

        min_width = 800
        if (abs(r2x - r1x) < min_width):
            print(abs(r2x - r1x))
            return mask

        extra_space_down = 30
        r1y += extra_space_down
        r2y += extra_space_down

        height = 140

        r3x = r2x
        r3y = r2y - height
        r4x = r1x
        r4y = r1y - height

        cv2.circle(classifier_mask, (int(r1x), int(r1y)), 20, (0, 0, 255), -1)
        cv2.circle(classifier_mask, (int(r2x), int(r2y)), 20, (0, 0, 255), -1)
        cv2.circle(classifier_mask, (int(r3x), int(r3y)), 20, (0, 0, 255), -1)
        cv2.circle(classifier_mask, (int(r4x), int(r4y)), 20, (0, 0, 255), -1)

        pts = np.array(
            [[r1x, r1y],
            [r2x, r2y],
            [r3x, r3y],
            [r4x, r4y]],
            dtype=np.int32
        ).reshape((-1, 1, 2))

        cv2.fillPoly(mask, [pts], color=255)

    return mask

def get_contours(hsv, lower_bounds, upper_bounds):
    # convert the hsv to a binary image by removing any pixels
    # that do not fall within the following HSV Min/Max values
    img_threshold = cv2.inRange(hsv, lower_bounds, upper_bounds)

    grow = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
    shrink = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    img_threshold = cv2.dilate(img_threshold, grow)
    img_threshold = cv2.erode(img_threshold, shrink)

    # find contours in the new binary image
    contours, _ = cv2.findContours(img_threshold,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # filter out specks
    min_area = 500
    filtered_contours = []
    for c in contours:
        if cv2.contourArea(c) >= min_area:
            filtered_contours.append(c)
    contours = filtered_contours

    # filter out outliers by y value
    if (len(contours) > 0):
        total_y = 0
        centroid_ys = []
        for c in contours:
            cy = centroid_y(c)
            centroid_ys.append(cy)
            total_y += cy
        mean_y = total_y / len(contours)

        max_dist_from_mean = 230
        filtered_contours = []
        for i in range(len(contours)):
            cy = centroid_ys[i]
            diff = abs(cy - mean_y)
            #print(diff)
            if (diff <= max_dist_from_mean):
                filtered_contours.append(contours[i])
        contours = filtered_contours

    return contours

def get_avg_angle_rad_between_sorted_contours(sorted_contours):
    pairs = len(sorted_contours) // 2
    if pairs == 0:
        return 0.0

    total_angle = 0.0
    n = len(sorted_contours)

    for i in range(pairs):
        fx, fy = centroid_xy(sorted_contours[i])
        lx, ly = centroid_xy(sorted_contours[n - 1 - i])
        total_angle += math.atan2(ly - fy, lx - fx)

    return total_angle / pairs

def rotate_contours(pivot, angle_rad, contours):
    cx, cy = pivot
    angle_deg = math.degrees(angle_rad)
    M = cv2.getRotationMatrix2D((cx, cy), angle_deg, 1.0)

    rotated = []
    for c in contours:
        if c is None or len(c) == 0:
            continue

        c_float = c.astype(np.float32)          # (N,1,2)
        rc = cv2.transform(c_float, M)          # float32

        # Limelight-safe: drawContours prefers int32 point contours
        rc = np.round(rc).astype(np.int32)

        if rc.shape[0] > 0:
            rotated.append(rc)

    return rotated
    cx, cy = pivot
    angle_deg = math.degrees(angle_rad)
    M = cv2.getRotationMatrix2D((cx, cy), angle_deg, 1.0)

    rotated = []
    for c in contours:
        if c is None or len(c) == 0:
            continue
        c_float = c.astype(np.float32)  # (N,1,2)
        rc = cv2.transform(c_float, M)
        if rc is not None and len(rc) > 0:
            rotated.append(rc)
    return rotated

def segment_contours(contours):
    single_ball_size = 100
    another_ball_threshold = 0.8

    segmented_rects = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)

        ratio = w / single_ball_size

        if (ratio > 1 + another_ball_threshold):
            split_amount = int(ratio)
            if (ratio % 1 > another_ball_threshold):
                split_amount += 1
            split_width = w / split_amount
            for i in range(split_amount):
                split_rect = x + i * split_width, y, split_width, h
                segmented_rects.append(split_rect)
        else:
            rect = x, y, w, h
            segmented_rects.append(rect)

    return segmented_rects

def rotate_rects(pivot, angle_rad, rects):
    cx, cy = pivot
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)

    rotated_rects = []

    for x, y, w, h in rects:
        # center of the rect
        rcx = x + w * 0.5
        rcy = y + h * 0.5

        # translate to pivot
        dx = rcx - cx
        dy = rcy - cy

        # rotate
        nx = dx * cos_a - dy * sin_a + cx
        ny = dx * sin_a + dy * cos_a + cy

        # convert back to top-left corner
        new_x = int(round(nx - w * 0.5))
        new_y = int(round(ny - h * 0.5))

        rotated_rects.append((new_x, new_y, w, h))

    return rotated_rects
# drawing functions========================
def show_sample_pixel(image, img_hsv, samplex, sampley):
    h, s, b = img_hsv[sampley, samplex]
    cv2.rectangle(image, (0, 60), (360, 120), (0, 0, 0), -1)
    cv2.putText(
        image, f"color: {h} {s} {b}", (20, 100),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0, (0, 255, 0), 2,
        cv2.LINE_AA
    )

def draw_contours(image, green_contours, purple_contours):
    if (len(green_contours) > 0):
        cv2.drawContours(image, green_contours, -1, (0, 255, 0), -1)
    if (len(purple_contours) > 0):
        cv2.drawContours(image, purple_contours, -1, (200, 0, 200), -1)

def draw_combined_contours(image, contours):
    contours = [c for c in contours if c is not None and len(c) > 0 and c.shape[0] > 0]
    if (len(contours) <= 0):
        return

    cv2.drawContours(image, contours, -1, (255, 255, 255), 2)
    """
    for i in range(len(contours)):
        contour = contours[i]

        x, y = centroid_xy(contour)
        cv2.putText(
            image, f"{i}", (int(x)-30, int(y)),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (0, 0, 0), 2,
            cv2.LINE_AA
        )
        """

def draw_rects(image, rects):
    for i in range(len(rects)):
        x, y, w, h = rects[i]
        x = int(x)
        y = int(y)
        w = int(w)
        h = int(h)
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 0), 3)
        cv2.rectangle(image, (x - 5, y - 50), (x + 20, y), (0, 0, 0), -1)
        cv2.putText(image, f"{i}", (x, y-20),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (255, 255, 255), 2, cv2.LINE_AA)

# utility functions=========================
def centroid_x(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return 0.0
    return M["m10"] / M["m00"]

def centroid_y(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return 0.0
    return M["m01"] / M["m00"]

def centroid_xy(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return 0.0, 0.0
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return cx, cy

def contour_at_point(x, y, size=6):
    # contour shape: (N, 1, 2) int32
    s = size
    pts = np.array([
        [x - s, y - s],
        [x + s, y - s],
        [x + s, y + s],
        [x - s, y + s],
    ], dtype=np.int32).reshape((-1, 1, 2))
    return pts

# returns length, (point1, point2)
def longest_2edges_from_approx(approx):
    pts = approx.reshape(-1, 2)  # (N, 2)
    n = len(pts)

    max_len = 0.0
    best_edge = None
    max_len2 = 0
    best_edge2 = None

    for i in range(n):
        p1 = pts[i]
        p2 = pts[(i + 1) % n]  # wrap around

        length = np.linalg.norm(p2 - p1)

        if length > max_len:
            max_len2 = length
            best_edge2 = best_edge
            max_len = length
            best_edge = (tuple(p1), tuple(p2))
        elif length > max_len2:
            max_len2 = length
            best_edge = (tuple(p1), tuple(p2))

    return max_len, best_edge, max_len2, best_edge2
 */
