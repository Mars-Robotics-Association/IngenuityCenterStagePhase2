package org.firstinspires.ftc.teamcode;

import java.util.function.Function;

public class MotionProfileAccelTrig extends MotionProfile {
    private final double maxAccel;
    private final double maxVelocity;

    private final double rampTime;
    private final double cruiseTime;
    private double rampDistance;

    public MotionProfileAccelTrig(double maxAccel, double maxVelocity, double totalDistance) {
        this.maxAccel = maxAccel;
        //this.rampDistance = trigRamp2ndIntegral(rampTime, maxAccel, maxVelocity);
        double rampDistance = maxVelocity * maxVelocity / maxAccel;
        if (totalDistance < (2 * this.rampDistance)) {
            maxVelocity = Math.sqrt(maxAccel * totalDistance / 2.0);
            rampDistance = maxVelocity * maxVelocity / maxAccel;
        }
        this.rampDistance = rampDistance;
        this.rampTime = 2 * maxVelocity / maxAccel;
        this.maxVelocity = maxVelocity;
        this.cruiseTime = (totalDistance - 2 * rampDistance) / maxVelocity;
    }

//    private static double trigRamp2ndIntegral(double x, double g, double t) {
//        return (g / 4.0) * (x * x + (t * t * Math.cos(2 * Math.PI * x / t) / 2.0 * Math.PI * Math.PI)) - (t * t * g / (8.0 * Math.PI * Math.PI));
//    }

    private static double trigRamp2ndIntegral(double x, double g, double s) {
        return (g / 4.0) * (x * x + (2.0 * s * s * Math.cos(Math.PI * g * x / s) / g * g * Math.PI * Math.PI)) - (s * s / (2.0 * g * Math.PI * Math.PI));
    }

    private double trigRamp2ndIntegral(double x) {
        return trigRamp2ndIntegral(x, maxAccel, maxVelocity);
    }

    private double trigRamp1stIntegral(double x) {
        double pigx = Math.PI * maxAccel * x;
        return (pigx - maxVelocity * Math.sin(pigx / maxVelocity)) / (2.0 * Math.PI);
    }

    private double trigRamp(double x) {
        return (maxAccel - maxAccel * Math.cos(2.0 * Math.PI * x / rampTime)) / 2.0;
    }

    private double rangeResult(double time, double middleValue, Function<Double, Double> callback) {
        if (time < rampTime) {
            return callback.apply(time);
        } else if (time > (cruiseTime + rampTime)) {
            return -callback.apply(time - rampTime - cruiseTime - rampTime);
        } else {
            return middleValue;
        }
    }

    @Override
    public double totalProfileTime() {
        return rampTime + cruiseTime + rampTime;
    }

    @Override
    public double profilePosition(double time) {
        return rangeResult(time, maxVelocity * time - rampDistance, x -> trigRamp2ndIntegral(x));
    }

    @Override
    public double profileVelocity(double time) {
        return rangeResult(time, maxVelocity, x -> trigRamp1stIntegral(x));
    }

    @Override
    public double profileAccel(double time) {
        return rangeResult(time, 0.0, x -> trigRamp(x));
    }
}
