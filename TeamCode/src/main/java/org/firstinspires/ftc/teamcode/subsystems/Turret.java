package org.firstinspires.ftc.teamcode.subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Component;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public final class Turret implements Component {
    public boolean isRedAlliance = true;
    private HardwareMap map;
    private Telemetry telemetry;
    private FtcDashboard dashboard;
    public DcMotorEx turretMotor;
    private PIDController pidController;
    public TurretState turretState;
    Pose2d targetPose = new Pose2d(-62, 62, 0);

    private ElapsedTime tagVisibleTimer = new ElapsedTime();
    private ElapsedTime tagLostTimer = new ElapsedTime();

    public static class Params{
        public double kP = 0.0075;
        public double kI = 0;
        public double kD = 0.0005;
        public int TURRET_INCREMENT = 60;
        public int TURRET_MAX = 300;
        public int TURRET_MIN = -300;
        public int TICKS_PER_REV = 1212;
        public int RIGHT_BOUND = -300;
        public int LEFT_BOUND = 300;
        public double TAG_LOCK_THRESHOLD = 1.0;
        public double TAG_LOST_THRESHOLD = 0.5;
    }

    public MecanumDrive drive;
    public Vision vision;
    public static Params TURRET_PARAMS = new Turret.Params();

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, MecanumDrive drive, Vision vision){
        this.map = hardwareMap;
//        this.telemetry = telemetry;
        this.drive = drive;
        this.vision = vision;

        this.dashboard = FtcDashboard.getInstance();
        this.telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        turretMotor = map.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController = new PIDController(TURRET_PARAMS.kP, TURRET_PARAMS.kI, TURRET_PARAMS.kD);
        turretState = TurretState.OFF;
    }

    public int getTurretEncoder() {
        return turretMotor.getCurrentPosition();
    }

    public void setTurretPosition(int ticks) {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double error = getTurretEncoder() - ticks;
        double power = pidController.updateWithError(error);
//        telemetry.addData("Turret Power", power);
        turretMotor.setPower(-power);
    }

    public void poseTargetToTurretTicks (Pose2d robotPose, Pose2d targetPose) {
        double deltaX = targetPose.position.x - robotPose.position.x;
        double deltaY = targetPose.position.y - robotPose.position.y;
        double turretMax = Math.toRadians(90);
        double turretMin = Math.toRadians(-90);

        double targetAngle = Math.atan2(deltaY, deltaX);
        double turretTargetAngle = targetAngle - robotPose.heading.toDouble();
        turretTargetAngle = Math.atan2(Math.sin(turretTargetAngle), Math.cos(turretTargetAngle));

        if (turretTargetAngle > turretMax)
            turretTargetAngle = Math.PI - turretTargetAngle; //mirror angle
        else if (turretTargetAngle < turretMin)
            turretTargetAngle = -Math.PI - turretTargetAngle;

        double inverseFactor = (isRedAlliance) ? 1 : -1;
        double turretTicksPerRadian = (TURRET_PARAMS.TICKS_PER_REV) / (2 * Math.PI) * 1;
        int targetTurretPosition = (int)(turretTargetAngle * turretTicksPerRadian);

//        telemetry.addData("Turret Angle", turretTargetAngle);
//        telemetry.addData("Turret Target", targetTurretPosition);
//        telemetry.addData("Turret Pose X", robotPose.position.x);
//        telemetry.addData("Turret Pose Y", robotPose.position.y);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret Angle", turretTargetAngle);
        packet.put("Turret Target", targetTurretPosition);
        packet.put("Turret Encoder", getTurretEncoder());

        dashboard.sendTelemetryPacket(packet);

        targetTurretPosition = Math.max(TURRET_PARAMS.RIGHT_BOUND, Math.min(targetTurretPosition, TURRET_PARAMS.LEFT_BOUND));
        setTurretPosition(targetTurretPosition);
    }

    private void fineAdjustTurretWithTag(AprilTagDetection tag) {
        if (tag == null) return;

        double tagX = tag.ftcPose.x;
        double tagZ = tag.ftcPose.z;

        double angularOffset = Math.atan2(-tagX, tagZ);
        int currentTicks = getTurretEncoder();

        double turretTicksPerRadian = (TURRET_PARAMS.TICKS_PER_REV) / (2 * Math.PI);
        int adjustment = (int)(angularOffset * turretTicksPerRadian);

        int targetTicks = currentTicks + adjustment;

        telemetry.addData("FINE ADJUST TARGET", targetTicks);
        telemetry.addData("TAG X (m)", tagX);
        telemetry.addData("TAG Z (m)", tagZ);
        telemetry.addData("ANGLE OFFSET (deg)", Math.toDegrees(angularOffset));

        targetTicks = Math.max(TURRET_PARAMS.RIGHT_BOUND, Math.min(targetTicks, TURRET_PARAMS.LEFT_BOUND));

        pidController.setTarget(0);
        double power = pidController.update(tagX);
        turretMotor.setPower(-power);
        telemetry.addData("POWWWWWWW", power);
//        setTurretPosition(targetTicks);
    }

    public enum TurretState {
        OFF, TRACKING, CENTER, COARSE, TAG_LOCK
    }

    @Override
    public void reset() {
    }

    @Override
    public void update(){
        int correctTagId = (isRedAlliance) ? 24 : 20;
        boolean isTagVisible = vision.isTagVisible() && vision.isCorrectTag(correctTagId);

        switch (turretState) {
            case OFF:
//                turretMotor.setPower(0);
                break;

            case TRACKING:
                poseTargetToTurretTicks(drive.localizer.getPose(), targetPose);
                break;

            case CENTER:
                setTurretPosition(0);
                break;

            case COARSE:
                poseTargetToTurretTicks(drive.localizer.getPose(), targetPose);

                if (isTagVisible) {
                    if (tagVisibleTimer.seconds() == 0)
                        tagVisibleTimer.reset();
                    tagLostTimer.reset();
                } else {
                    tagVisibleTimer.reset();
                }

                if (tagVisibleTimer.seconds() >= TURRET_PARAMS.TAG_LOCK_THRESHOLD) {
                    turretState = TurretState.TAG_LOCK;
                    tagVisibleTimer.reset();
                }
                break;

            case TAG_LOCK:
                fineAdjustTurretWithTag(vision.getCurrentTag());

                if (!isTagVisible) {
                    if (tagLostTimer.seconds() == 0) tagLostTimer.reset();
                } else {
                    tagLostTimer.reset();
                }

                if (tagLostTimer.seconds() >= TURRET_PARAMS.TAG_LOST_THRESHOLD) {
                    turretState = TurretState.COARSE;
                    tagLostTimer.reset();
                }
                break;
        }

        if (isRedAlliance)
            targetPose = new Pose2d(-62, 62, 0);
        else
            targetPose = new Pose2d(-62, -62, 0);

//        telemetry.addData("Alliance", isRedAlliance ? "Red" : "Blue");
//        telemetry.addData("Turret State", turretState.toString());
//        telemetry.addData("Turret Encoder", getTurretEncoder());
//        telemetry.addData("Is Tag Visible", isTagVisible);
//        telemetry.addData("TAG VISIBLE TIMER", tagVisibleTimer.seconds());
//        telemetry.addData("TAG LOST TIMER", tagLostTimer.seconds());

//        if (!vision.tagProcessor.getDetections().isEmpty()){
//            telemetry.addData("FTC X", vision.getCurrentTag().ftcPose.x);
//        }

        pidController.setPIDValues(TURRET_PARAMS.kP, TURRET_PARAMS.kI, TURRET_PARAMS.kD);
    }

    @Override
    public String test(){
        return null;
    }
}
