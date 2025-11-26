package org.firstinspires.ftc.teamcode.utils.autoHelpers;

public class AutoRobotStatus {
    public boolean preCollecting;
    public boolean collecting;
    public boolean postCollecting;
    public void cycle() {
        if(preCollecting) {
            preCollecting = false;
            collecting = true;
        }
        else if(collecting) {
            collecting = false;
            postCollecting = true;
        }
        else if(postCollecting)
            postCollecting = false;
    }
}
