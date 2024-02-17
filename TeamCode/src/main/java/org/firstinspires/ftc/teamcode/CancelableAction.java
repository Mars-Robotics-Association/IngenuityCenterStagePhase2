package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;

public abstract class CancelableAction implements Action {
    private boolean canceled = false;

    protected boolean isCanceled() {
        return canceled;
    }

    public void cancel() {
        canceled = true;
    }
}
