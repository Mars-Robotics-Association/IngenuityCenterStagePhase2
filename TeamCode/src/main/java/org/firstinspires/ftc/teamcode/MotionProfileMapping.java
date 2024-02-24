package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotionProfileMapping {
    private final MotionProfile motionProfile;
    private final double signum;
    private final double startedSeconds;
    private final double Ks, Kv, Kp, Ki;
    private int positionTolerance = 20;
    private double velocityTolerance = 100;
    private final int targetPosition;
    private final Telemetry telemetry;
    private final int startingPosition;
    private final int totalDistance;
    private final double startingVelocity;

    private int accumulator;

    MotionProfileMapping(Telemetry telemetry, double maxAccel, double maxVelocity, int currentPosition, double currentVelocity, int targetPosition, double currentSeconds, double Ks, double Kv, double Kp, double Ki) {
        this.signum = Math.signum(targetPosition - currentPosition);
        totalDistance = Math.abs(targetPosition - currentPosition);
        startingVelocity = signum * currentVelocity;
        this.motionProfile = new MotionProfileAccelTrig(Math.abs(maxAccel), Math.abs(maxVelocity), totalDistance, startingVelocity);
        this.startedSeconds = currentSeconds;
        this.Ks = Ks;
        this.Kv = Kv;
        this.Kp = Kp;
        this.Ki = Ki;
        accumulator = 0;
        this.startingPosition = currentPosition;
        this.targetPosition = targetPosition;
        this.telemetry = telemetry;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public double calculate(int position, double currentSeconds) {
        //telemetry.addData("planned distance", totalDistance);
        //telemetry.addData("starting velocity", startingVelocity);
        double beforeTarget = (signum < 0 && position < targetPosition) || (position > targetPosition) ? 0.0 : 1.0;
        double time = currentSeconds - startedSeconds;
        double setPoint = signum * motionProfile.profilePosition(time) + startingPosition;
        telemetry.addData("expected profile time", motionProfile.totalProfileTime());
        telemetry.addData("motion profile setpoint", setPoint);
        double error = setPoint - position;
        accumulator = (int) (accumulator * 0.65 + error);
        double v = signum * motionProfile.profileVelocity(time);
        telemetry.addData("motion profile velocity", v);
        return beforeTarget * Math.signum(v) * Ks +
                beforeTarget * Kv * v +
                Kp * error +
                Ki * accumulator;
    }

    public void setTolerances(int positionTolerance, double velocityTolerance) {
        this.positionTolerance = Math.abs(positionTolerance);
        this.velocityTolerance = Math.abs(velocityTolerance);
    }

    public boolean isSettled(int position, double velocity) {
        return Math.abs(position - targetPosition) <= positionTolerance && Math.abs(velocity) <= velocityTolerance;
    }

    public boolean inProfile(double currentSeconds) {
        if (motionProfile == null) {
            return false;
        } else {
            return motionProfile.insideProfile(currentSeconds);
        }
    }
}
