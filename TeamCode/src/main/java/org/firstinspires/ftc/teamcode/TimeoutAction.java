package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;

public class TimeoutAction implements Action {
    private final CancelableAction cancelableAction;
    private final SleepAction sleepAction;

    public TimeoutAction(CancelableAction action, double timeoutSeconds){
        cancelableAction = action;
        sleepAction = new SleepAction(timeoutSeconds);
    }

    public boolean run(@NonNull TelemetryPacket packet){
        if(!sleepAction.run(packet)){
            cancelableAction.cancel();
        }

        return cancelableAction.run(packet);
    }
}
