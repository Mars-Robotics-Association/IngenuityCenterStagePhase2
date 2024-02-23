package org.firstinspires.ftc.teamcode;

public class MotionProfileMapping {
    private final MotionProfile motionProfile;
    private final double signum;
    private final double startedSeconds;
    private final double Ks, Kv, Kp, Ki;

    private int accumulator;

    MotionProfileMapping(double maxAccel, double maxVelocity, int currentPosition, double currentVelocity, int targetPosition, double currentSeconds, double Ks, double Kv, double Kp, double Ki) {
        this.motionProfile = new MotionProfileAccelTrig(Math.abs(maxAccel), Math.abs(maxVelocity), Math.abs(targetPosition - currentPosition), currentVelocity);
        this.startedSeconds = currentSeconds;
        this.signum = Math.signum(targetPosition - currentPosition);
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
        return signum * Ks +
                signum * Kv * motionProfile.profileVelocity(time) +
                signum * Kp * error +
                signum * Ki * accumulator;
    }
}
