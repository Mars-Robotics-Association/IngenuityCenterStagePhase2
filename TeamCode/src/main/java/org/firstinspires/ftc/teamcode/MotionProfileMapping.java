package org.firstinspires.ftc.teamcode;

public class MotionProfileMapping {
    private final MotionProfile motionProfile;
    private final double signum;
    private final double startedSeconds;
    private final double Ks, Kv, Kp, Ki;
    private int positionTolerance = 20;
    private double velocityTolerance = 100;

    private int accumulator;

    MotionProfileMapping(double maxAccel, double maxVelocity, int currentPosition, double currentVelocity, int targetPosition, double currentSeconds, double Ks, double Kv, double Kp, double Ki) {
        this.signum = Math.signum(targetPosition - currentPosition);
        this.motionProfile = new MotionProfileAccelTrig(Math.abs(maxAccel), Math.abs(maxVelocity), Math.abs(targetPosition - currentPosition), signum * currentVelocity);
        this.startedSeconds = currentSeconds;
        this.Ks = Ks;
        this.Kv = Kv;
        this.Kp = Kp;
        this.Ki = Ki;
        accumulator = 0;
    }

    public double calculate(int position, double currentSeconds) {
        double time = currentSeconds - startedSeconds;
        double setPoint = motionProfile.profilePosition(time);
        double error = setPoint - position;
        accumulator = (int) (accumulator * 0.8 + error);
        double v = signum * motionProfile.profileVelocity(time);
        return Math.signum(v) * Ks +
                Kv * v +
                signum * Kp * error +
                signum * Ki * accumulator;
    }

    public void setTolerances(int positionTolerance, double velocityTolerance) {
        this.positionTolerance = Math.abs(positionTolerance);
        this.velocityTolerance = Math.abs(velocityTolerance);
    }

    public boolean isSettled(int position, double velocity) {
        return Math.abs(position) <= positionTolerance && Math.abs(velocity) <= velocityTolerance;
    }

    public boolean inProfile(double currentSeconds) {
        return (currentSeconds - startedSeconds) <= motionProfile.totalProfileTime();
    }
}
