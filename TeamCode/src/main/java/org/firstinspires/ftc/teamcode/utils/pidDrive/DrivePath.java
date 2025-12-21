package org.firstinspires.ftc.teamcode.utils.pidDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.PIDController;
import org.firstinspires.ftc.teamcode.utils.misc.TelemetryHelper;

import java.util.ArrayList;
import java.util.Arrays;

public class DrivePath implements Action {
    public static double moveRightSign = -1, moveForwardSign = 1, turnLeftSign = -1;
    private final MecanumDrive drivetrain;
    private final PinpointLocalizer odo;
    private final Telemetry telemetry;
    private final ArrayList<Waypoint> waypoints; // list of all waypoints in drive path
    private int curWaypointIndex; // the waypoint index the drivetrain is currently trying to go to
    private PIDController totalDistancePID, waypointDistancePID, headingRadErrorPID;
    private final ElapsedTime waypointTimer;
    private boolean first;
    private Pose2d startPose;

    public DrivePath(MecanumDrive drivetrain, Waypoint ...waypoints) {
        this(drivetrain, null, waypoints);
    }
    public DrivePath(MecanumDrive drivetrain, Telemetry telemetry, Waypoint ...waypoints) {
        this.drivetrain = drivetrain;
        this.odo = drivetrain.pinpoint();
        this.telemetry = telemetry;

        this.waypoints = new ArrayList<>();
        this.waypoints.addAll(Arrays.asList(waypoints));
        curWaypointIndex = 0;

        waypointTimer = new ElapsedTime();
        first = true;
        startPose = new Pose2d(0, 0, 0);
    }

    public void addWaypoint(Waypoint waypoint) {
        waypoint.setDistToNextWaypoint(waypoints.get(waypoints.size() - 1));
        waypoints.add(waypoints.size() - 1, waypoint);
        if(waypoints.size() >= 3)
            waypoints.get(waypoints.size() - 3).setDistToNextWaypoint(waypoint);
    }
    public Waypoint getWaypoint(int index) {
        return waypoints.get(index);
    }
    private Waypoint getCurWaypoint() {
        return waypoints.get(Math.min(waypoints.size() - 1, curWaypointIndex));
    }
    private PathParams getCurParams() {
        return getCurWaypoint().params;
    }
    private double getWaypointDistanceToTarget(int waypointIndex) {
        if (waypointIndex == waypoints.size() - 1)
            return 0;
        double dist = 0;
        for (int i=waypointIndex; i<waypoints.size() - 1; i++)
            dist += waypoints.get(i).getDistToNextWaypoint();
        return dist;
    }
    private Vector2d updateTargetDir(double robotX, double robotY, double headingRad) {
        // translating target so that drivetrain is around origin
        double xFromRobot = getCurWaypoint().x() - robotX; // 48
        double yFromRobot = getCurWaypoint().y() - robotY; // 0
        // rotating target around origin
        double rotatedXFromRobot = xFromRobot * Math.sin(headingRad) - yFromRobot * Math.cos(headingRad);
        double rotatedYFromRobot = xFromRobot * Math.cos(headingRad) + yFromRobot * Math.sin(headingRad);
        // 48 * sin(0) - 0 * cos(0) = 0
        // 48 * cos(0) + 0 * sin(0) = 48
        // translating target back to absolute; this returns the direction to the next waypoint IN THE ROBOT'S COORDINATE PLANE
        Vector2d targetDir = new Vector2d(rotatedXFromRobot, rotatedYFromRobot);
        double targetDirMag = Math.hypot(targetDir.x, targetDir.y);
        return targetDir.div(targetDirMag); // normalize
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        drivetrain.updatePoseEstimate();
        Pose2d pose = odo.getPose();
        double rx = pose.position.x, ry = pose.position.y, rHeadingRad = MathUtils.correctRad(pose.heading.toDouble()), rHeadingDeg = Math.toDegrees(rHeadingRad);

        if (first) {
            first = false;
            resetToNewWaypoint();
            startPose = new Pose2d(pose.position, pose.heading);
        }

        // finding direction that motor powers should be applied in
        Vector2d targetDir = updateTargetDir(rx, ry, rHeadingRad);

        // note: error is calculated in field's coordinate plane
        double xWaypointError = Math.abs(rx - getCurWaypoint().x());
        double yWaypointError = Math.abs(ry - getCurWaypoint().y());

        double headingDegWaypointError = getCurWaypoint().headingDeg() - rHeadingDeg;
        // flip heading error if necessary
        double absHeadingWaypointError = Math.abs(headingDegWaypointError);
        boolean flipHeadingDirection = absHeadingWaypointError > 180;
        if (flipHeadingDirection)
            headingDegWaypointError = Math.signum(headingDegWaypointError) * (360 - absHeadingWaypointError);

        // tolerance
        boolean inPositionTolerance = xWaypointError <= getCurWaypoint().tolerance.xTol && yWaypointError <= getCurWaypoint().tolerance.yTol;
        boolean inHeadingTolerance = Math.abs(headingDegWaypointError) <= getCurWaypoint().tolerance.headingDegTol;
        // pass position
        if(getCurParams().passPosition && getDotProductToNextWaypoint(pose) > 0)
                inPositionTolerance = true;

        boolean inWaypointTolerance = inPositionTolerance && inHeadingTolerance;
        boolean reachedMaxTime = getCurParams().hasMaxTime() && waypointTimer.seconds() >= getCurParams().maxTime;

        // in tolerance
        if (inWaypointTolerance || reachedMaxTime || getCurParams().customEndCondition.getAsBoolean()) {
            curWaypointIndex++;
            // completely finished drive path
            if (curWaypointIndex >= waypoints.size()) {
                drivetrain.stop();
                return false;
            }
            // finished current waypoint path, moving on to next waypoint
            else {
                // set new PID targets
                resetToNewWaypoint();

                // recalculate new waypoint errors
                xWaypointError = Math.abs(rx - getCurWaypoint().x());
                yWaypointError = Math.abs(ry - getCurWaypoint().y());
                headingDegWaypointError = getCurWaypoint().headingDeg() - rHeadingDeg;
                absHeadingWaypointError = Math.abs(headingDegWaypointError);
                flipHeadingDirection = absHeadingWaypointError > 180;
                if (flipHeadingDirection)
                    headingDegWaypointError = Math.signum(headingDegWaypointError) * (360 - absHeadingWaypointError);

                // recalculate new tolerances
                inPositionTolerance = xWaypointError <= getCurWaypoint().tolerance.xTol && yWaypointError <= getCurWaypoint().tolerance.yTol;
                inHeadingTolerance = Math.abs(headingDegWaypointError) <= getCurWaypoint().tolerance.headingDegTol;
            }
        }

        // calculate inputs to speed PIDs
        double waypointDistAway = Math.sqrt(xWaypointError * xWaypointError + yWaypointError * yWaypointError);
        double totalDistanceAway = waypointDistAway + getWaypointDistanceToTarget(curWaypointIndex);

        // calculate translational speed
        double speed = 0;
        if (!inPositionTolerance) {
            double a = Math.abs(totalDistancePID.update(totalDistanceAway));
            double b = Math.abs(waypointDistancePID.update(waypointDistAway));
            double t = getCurParams().slowDownPercent;
            speed = a + (b - a) * t;
            speed = Range.clip(speed, getCurParams().minLinearPower, getCurParams().maxLinearPower);
        }
        double lateralPower = targetDir.x * speed * getCurParams().lateralWeight * moveRightSign;
        double axialPower = targetDir.y * speed * getCurParams().axialWeight * moveForwardSign;

        // calculate angular speed (heading)
        double headingPower = 0;
        if (!inHeadingTolerance) {
            headingPower = headingRadErrorPID.update(headingDegWaypointError);
            double headingSign = Math.signum(headingPower);
            headingPower = -headingSign * Range.clip(Math.abs(headingPower), getCurParams().minHeadingPower, getCurParams().maxHeadingPower);
            if (flipHeadingDirection)
                headingPower *= -1;
        }
        headingPower *= turnLeftSign;
        
        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(axialPower, lateralPower), headingPower));
        /*
        robot.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));
         */
        
        if (telemetry != null) {
            telemetry.addData("current position", MathUtils.format3(rx) + " ," + MathUtils.format3(ry) + ", " + MathUtils.format3(rHeadingDeg));
            telemetry.addData("target position", MathUtils.format3(getCurWaypoint().x()) + " ," + MathUtils.format3(getCurWaypoint().y()) + ", " + MathUtils.format3(getCurWaypoint().headingDeg()));
            telemetry.addData("target dir", MathUtils.format3(targetDir.x) + ", " + MathUtils.format3(targetDir.y));
            telemetry.addData("waypoint errors", MathUtils.format3(xWaypointError) + ", " + MathUtils.format3(yWaypointError) + ", " + MathUtils.format3(headingDegWaypointError));
            telemetry.addData("in position tolerance", inPositionTolerance);
            telemetry.addData("in heading tolerance", inHeadingTolerance);
            telemetry.addLine();
            telemetry.addData("speed", MathUtils.format3(speed));
            telemetry.addData("powers (lat, ax, heading)", MathUtils.format3(lateralPower) + ", " + MathUtils.format3(axialPower) + ", " + MathUtils.format2(headingPower));
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose2d prevWaypointPose = curWaypointIndex == 0 ? startPose : getWaypoint(curWaypointIndex - 1).pose;
            TelemetryHelper.radii[0] = 5;
            TelemetryHelper.radii[1] = 5;
            TelemetryHelper.radii[2] = 5;
            TelemetryHelper.numPosesToShow = 3;
            TelemetryHelper.addRobotPoseToCanvas(fieldOverlay, pose, prevWaypointPose, getCurWaypoint().pose);
            fieldOverlay.setStroke("black");
            fieldOverlay.strokeLine(pose.position.x, pose.position.y, pose.position.x + targetDir.x * 10, pose.position.y + targetDir.y * 10);

            telemetry.update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        return true;
    }

    // calculates vector from previous waypoint to next waypoint
    // calculates vector from next waypoint to robot
    // returns the dot product of these 2 vectors
    // if dot product is positive, robot overshot waypoint
    // if dot is negative, robot has not yet overshot waypoint
    private double getDotProductToNextWaypoint(Pose2d pose) {
        Vector2d oldPosition = curWaypointIndex == 0 ? startPose.position : waypoints.get(curWaypointIndex - 1).pose.position;
        Vector2d targetWaypointPosition = getCurWaypoint().pose.position;

        Vector2d relativeWaypoint = targetWaypointPosition.minus(oldPosition);
        Vector2d relativePositionToWaypoint = pose.position.minus(targetWaypointPosition);
        return relativeWaypoint.x * relativePositionToWaypoint.x + relativeWaypoint.y * relativePositionToWaypoint.y;
    }

    private void resetToNewWaypoint() {
        waypointTimer.reset();

        totalDistancePID = new PIDController(getCurParams().speedKp, getCurParams().speedKi, getCurParams().speedKd);
        totalDistancePID.reset();
        totalDistancePID.setTarget(0);
        totalDistancePID.setOutputBounds(0, 1);
        waypointDistancePID = new PIDController(getCurParams().speedKp, getCurParams().speedKi, getCurParams().speedKd);
        waypointDistancePID.reset();
        waypointDistancePID.setTarget(0);
        waypointDistancePID.setOutputBounds(0, 1);

        headingRadErrorPID = new PIDController(getCurParams().headingKp, getCurParams().headingKi, getCurParams().headingKd);
        headingRadErrorPID.reset();
        headingRadErrorPID.setTarget(0);
        headingRadErrorPID.setOutputBounds(-1, 1);
    }
}

