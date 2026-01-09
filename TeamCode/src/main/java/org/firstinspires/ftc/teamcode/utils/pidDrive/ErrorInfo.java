package org.firstinspires.ftc.teamcode.utils.pidDrive;

public class ErrorInfo {
    public final double xError, yError, distanceError, headingDegError, headingPowerDirFlip;
    public ErrorInfo(double xError, double yError, double distanceError, double headingDegError, double headingPowerDirFlip) {
        this.xError = xError;
        this.yError = yError;
        this.distanceError = distanceError;
        this.headingDegError = headingDegError;
        this.headingPowerDirFlip = headingPowerDirFlip;
    }
}
