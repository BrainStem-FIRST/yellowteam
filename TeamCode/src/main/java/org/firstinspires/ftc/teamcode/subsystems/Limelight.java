
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.utils.math.Vec;

public class Limelight extends Component {
    // i should tune the camera so that it gives me the turret center position
    private final Limelight3A limelight;
    private Vec turretPos;
    private double turretHeading;
    private Vec robotPos;
    private double robotHeading;
    private Vec robotTurretVec;
    private LLResult result;
    public Limelight(HardwareMap hardwareMap, Telemetry telemetry, BrainSTEMRobot robot) {
        super(hardwareMap, telemetry, robot);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        turretPos = new Vec(0, 0);
        robotPos = new Vec(0, 0);
        robotTurretVec = new Vec(0, 0);
    }

    @Override
    public void printInfo() {
        telemetry.addLine("LIMELIGHT");
        if(result != null) {
            telemetry.addData("isValid", result.isValid());
            telemetry.addData("turret pos", turretPos);
            telemetry.addData("turret heading", Math.floor(turretHeading * 180 / Math.PI * 100) / 100);
            telemetry.addData("robot pos", robotPos);
            telemetry.addData("robot heading", Math.floor(robotHeading * 180 / Math.PI * 100) / 100);
        }
        else
            telemetry.addLine("result is null");
    }

    @Override
    public void reset() {}


    @Override
    public void update() {
        result = limelight.getLatestResult();
        if (result == null) {
            robotPos = null;
            robotHeading = 0;
            return;
        }

        Pose3D turretPose = result.getBotpose();

        telemetry.addLine("LIMELIGHT UPDATE=================");
        if(turretPose.getPosition().x != 0 && turretPose.getPosition().y != 0) {
            Position temp = turretPose.getPosition().toUnit(DistanceUnit.INCH);
            this.turretPos = new Vec(temp.x, temp.y);
            turretHeading = turretPose.getOrientation().getYaw(AngleUnit.RADIANS);

            int currentTurretPosition = robot.turret.turretMotor.getCurrentPosition();
            double relTurretAngleRad = Turret.getTurretRelativeAngleRad(currentTurretPosition);
            robotHeading = turretHeading - relTurretAngleRad;
            telemetry.addData("rel turret angle deg", Math.floor(relTurretAngleRad * 180 / Math.PI));
            if(robotHeading > Math.PI)
                robotHeading -= Math.PI * 2;
            robotTurretVec = new Vec(Turret.offsetFromCenter * Math.cos(robotHeading), Turret.offsetFromCenter * Math.sin(robotHeading));
            robotPos = turretPos.add(robotTurretVec);
            telemetry.addData("robot pose", getRobotPos().x + ", " +  getRobotPos().y + " | " + Math.toDegrees(robotHeading));
        }
        else
            telemetry.addData("NO POSE FOUND - robot pose", getRobotPos().x + ", " +  getRobotPos().y + " | " + Math.toDegrees(robotHeading));
    }

    // robotHeading should be in radians
    // robot turret offset is distance from center of turret to center of robot
    public Vec getRobotPos() {
        return robotPos;
    }
    public Pose2d getRobotPose() {
        if(robotPos == null)
            return null;
        return new Pose2d(robotPos.x, robotPos.y, robotHeading);
    }
    public double getRobotHeading() {
        return robotHeading;
    }
    public Vec getTurretPos() {
        return turretPos;
    }
    public double getTurretHeading() {
        return turretHeading;
    }
}

