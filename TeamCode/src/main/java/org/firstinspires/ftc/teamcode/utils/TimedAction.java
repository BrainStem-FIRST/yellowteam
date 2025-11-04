package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TimedAction implements Action {
    private final ElapsedTime timer;
    private final Action action;
    private final double maxTime;
    private boolean first = true;
    public TimedAction(Action action, double maxTime) {
        this.action = action;
        this.maxTime = maxTime;
        this.timer = new ElapsedTime();
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (first) {
            first = false;
            timer.reset();
        }

        boolean keepGoing = action.run(telemetryPacket);
        boolean outOfTime = timer.seconds() > maxTime;
        return keepGoing && !outOfTime;
    }
}
