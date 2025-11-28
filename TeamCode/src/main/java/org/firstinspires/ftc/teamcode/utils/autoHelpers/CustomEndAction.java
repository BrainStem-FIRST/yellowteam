package org.firstinspires.ftc.teamcode.utils.autoHelpers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class CustomEndAction implements Action {
    private final ElapsedTime timer;
    private final Action action;
    private final double maxTime;
    private final BooleanSupplier endCondition;
    private boolean first = true;
    public Runnable onEnd;
    public CustomEndAction(Action action, BooleanSupplier endCondition, double maxTime) {
        this.action = action;
        this.endCondition = endCondition;
        this.maxTime = maxTime;
        this.timer = new ElapsedTime();
        onEnd = () -> {};
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (first) {
            first = false;
            timer.reset();
        }

        boolean keepGoing = action.run(telemetryPacket);
        boolean outOfTime = timer.seconds() > maxTime;
        return keepGoing && !outOfTime && !endCondition.getAsBoolean();
    }

    public CustomEndAction setEndFunction(Runnable endFunction) {
        CustomEndAction newAction = new CustomEndAction(action, endCondition, maxTime);
        newAction.onEnd = endFunction;
        return newAction;
    }
}
