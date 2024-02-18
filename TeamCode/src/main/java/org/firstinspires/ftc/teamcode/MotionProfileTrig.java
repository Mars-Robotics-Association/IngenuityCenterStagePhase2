package org.firstinspires.ftc.teamcode;

public class MotionProfileTrig extends MotionProfile {
    private final double maxAccel;
    private final double maxVelocity;
    private final double totalDistance;
    private final double rampTime;
    private final double cruiseTime;
    private final double rampDistance;

    public MotionProfileTrig(double maxAccel, double maxVelocity, double totalDistance) {
        this.maxAccel = maxAccel;
        this.maxVelocity = maxVelocity;
        this.totalDistance = totalDistance;
        this.rampTime = (Math.PI * maxVelocity) / (2 * maxAccel);
        this.rampDistance = trigRampIntegral(rampTime, maxVelocity, rampTime);
        this.cruiseTime = (totalDistance - 2 * rampDistance) / maxVelocity;
    }

    private static double trigRampIntegral(double x, double s, double t) {
        double pix = Math.PI * x;
        return (s * (pix - t * Math.sin(pix / t))) / (2.0 * Math.PI);
    }

    private double trigRampIntegral(double x) {
        return trigRampIntegral(x, maxVelocity, rampTime);
    }

    private double trigRamp(double x) {
        return (maxVelocity - maxVelocity * Math.cos(Math.PI * x / rampTime)) / 2.0;
    }

    private double trigRampDeriv(double x) {
        return (maxVelocity * Math.PI * Math.sin(Math.PI * maxVelocity / rampTime)) / (2.0 * rampTime);
    }

    @Override
    public double totalProfileTime() {
        return rampTime + cruiseTime + rampTime;
    }

    @Override
    public double profileVelocity(double time) {
        if (time < rampTime) {
            return trigRamp(time);
        } else if (time > (cruiseTime + rampTime)) {
            return trigRamp(time - cruiseTime);
        } else {
            return maxVelocity;
        }
    }

    @Override
    public double profilePosition(double time) {
        if (time < rampTime) {
            return trigRampIntegral(time);
        } else if (time > (cruiseTime + rampTime)) {
            return rampDistance + maxVelocity * (time - rampTime) + rampDistance - trigRampIntegral(rampTime - time + cruiseTime + rampTime);
        } else {
            return rampDistance + maxVelocity * (time - rampTime);
        }
    }

    @Override
    public double profileAccel(double time) {
        if (time < rampTime) {
            return trigRampDeriv(time);
        } else if (time > (cruiseTime + rampTime)) {
            return trigRampDeriv(time - cruiseTime);
        } else {
            return 0;
        }
    }
}