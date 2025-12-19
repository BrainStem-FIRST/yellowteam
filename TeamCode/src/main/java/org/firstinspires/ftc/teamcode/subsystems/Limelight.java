
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utils.math.MathUtils;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;

import java.util.ArrayList;

@Config
public class Limelight extends Component {
    public enum UpdatePoseType {
        CONTINUOUS,
        ON_COMMAND
    }
    public enum UpdateState {
        OFF,
        PASSIVE_READING,
        UPDATING_POSE
    }
    public static class UpdatePoseParams {
        public double maxUpdateTranslationalVel = 0.5, maxUpdateHeadingDegVel = 2; // inches and degrees
        public int maxUpdateTurretVelTicksPerSec = 1;
        public double maxUpdateDist = 110;
        public int numPrevFramesToAvg = 4;
        public int minTimeBetweenUpdates = 5;
    }
    public static UpdatePoseType updatePoseType = UpdatePoseType.CONTINUOUS;
    public static UpdateState offUpdateState = UpdateState.PASSIVE_READING;
    public static UpdatePoseParams updatePoseParams = new UpdatePoseParams();
    // i should tune the camera so that it gives me the turret center position
    private final Limelight3A limelight;
    private Pose2d turretPose, robotPose;
    private Vector2d robotTurretVec;
    private LLResult result;
    private Pose2d lastAvgTurretPose;
    private final ArrayList<Pose3D> lastTurretPoses;
    public double maxTranslationalError, maxHeadingErrorDeg;
    private boolean canUpdateReliably;
    public boolean successfullyFoundPose;
    private UpdateState updateState;
    private OdoInfo odoVel = new OdoInfo(0, 0, 0);
    private Pose2d odoPose = new Pose2d(0, 0, 0);
    private int numSetPoses = 0;
    private double lastUpdateTimeMs = 0;
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
        setState(UpdateState.PASSIVE_READING);
        successfullyFoundPose = false;
        maxTranslationalError = 0;
        maxHeadingErrorDeg = 0;
    }

    @Override
    public void printInfo() {
        telemetry.addLine("LIMELIGHT");
        telemetry.addData("state", updateState);
        if(result != null) {
            telemetry.addData("num set poses", numSetPoses);
            telemetry.addData("   isValid", result.isValid());
            telemetry.addData("   can update reliably", canUpdateReliably);
            telemetry.addData("   successfully found pose", successfullyFoundPose);
            telemetry.addData("   last update time", MathUtils.format2(lastUpdateTimeMs * 0.001));
            telemetry.addLine();
            telemetry.addData("   max translational error", maxTranslationalError);
            telemetry.addData("   max heading error", maxHeadingErrorDeg);
            telemetry.addData("   turret pose", MathUtils.format2(turretPose.position.x) + " " + MathUtils.format2(turretPose.position.y) + " " + MathUtils.format2(Math.toDegrees(turretPose.heading.toDouble())));
            telemetry.addData("   robot pose", MathUtils.format2(robotPose.position.x) + " " + MathUtils.format2(robotPose.position.y) + " " + MathUtils.format2(Math.toDegrees(robotPose.heading.toDouble())));

            for (int i=0; i<Math.min(3, lastTurretPoses.size()); i++)
                telemetry.addData("   last pose " + (i + 1),
                          MathUtils.format2(lastTurretPoses.get(i).getPosition().x) + " " +
                                MathUtils.format2(lastTurretPoses.get(i).getPosition().y) + " " +
                                MathUtils.format2(lastTurretPoses.get(i).getOrientation().getYaw(AngleUnit.DEGREES)));

            telemetry.addData("odo vel", odoVel);
            telemetry.addData("odo pose", MathUtils.format2Pose(odoPose));
            telemetry.addData("turret vel", robot.turret.turretMotor.getVelocity());
        }
        else
            telemetry.addLine("   result is null");
    }

    @Override
    public void update() {
        boolean prevCanUpdateReliably = canUpdateReliably;
        canUpdateReliably = canUpdateReliably();
        double curTimeMs = System.currentTimeMillis();
        if (!canUpdateReliably) {
            setState(offUpdateState);
            successfullyFoundPose = false;
            lastTurretPoses.clear();
        }
        else if (!prevCanUpdateReliably && updatePoseType == UpdatePoseType.CONTINUOUS && curTimeMs - lastUpdateTimeMs > updatePoseParams.minTimeBetweenUpdates) {
            setState(UpdateState.UPDATING_POSE);
            lastUpdateTimeMs = curTimeMs;
        }

        switch (updateState) {
            case OFF:
                break;
            case PASSIVE_READING:
                robotPose = updatePoseFromCamera();
                if (robotPose == null)
                    robotPose = new Pose2d(0, 0, 0);
                break;
            case UPDATING_POSE:
                robotPose = updatePoseFromCamera();
                updateMaxErrors(lastAvgTurretPose, turretPose);

                successfullyFoundPose = robotPose != null;
                if (robotPose == null)
                    robotPose = new Pose2d(0, 0, 0);

                if(successfullyFoundPose) {
                    robot.drive.stop();
                    robot.drive.localizer.setPose(robotPose);
                    numSetPoses++;
                    setState(offUpdateState);
                }
                break;
        }
    }
    private Pose2d updatePoseFromCamera() {
        lastAvgTurretPose = new Pose2d(turretPose.position, turretPose.heading);

        result = limelight.getLatestResult();
        if (result == null)
            return null;

        Pose3D curFrameTurretPose = result.getBotpose();
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
    private boolean canUpdateReliably() {
        odoVel = robot.drive.pinpoint().getMostRecentVelocity();
        odoPose = robot.drive.localizer.getPose();
        Pose2d targetPose = robot.turret.targetPose;
        boolean turretSlowEnough = robot.turret.turretMotor.getVelocity() < updatePoseParams.maxUpdateTurretVelTicksPerSec;
        boolean slowEnough = Math.abs(Math.toDegrees(odoVel.headingRad)) < updatePoseParams.maxUpdateHeadingDegVel && Math.hypot(odoVel.x, odoVel.y) < updatePoseParams.maxUpdateTranslationalVel;
        boolean inRange = Math.hypot(odoPose.position.x - targetPose.position.x, odoPose.position.y - targetPose.position.y) < updatePoseParams.maxUpdateDist;
        return turretSlowEnough && slowEnough && inRange;
    }

    public UpdateState getState() {
        return updateState;
    }
    public void setState(UpdateState state) {
        if (state == this.updateState)
            return;

        this.updateState = state;
        if (state == UpdateState.UPDATING_POSE) {
            lastTurretPoses.clear();
            successfullyFoundPose = false;
        }

    }
}

