package org.firstinspires.ftc.teamcode;

import java.util.function.BiFunction;
import java.util.function.Function;

public class MotionProfileAccelTrig extends MotionProfile {
    private final double maxAccel;
    private final double maxVelocity;
    private final double rampUpTime;
    private final double rampDownTime;
    private final double rampUpDistance;
    private final double rampDownDistance;
    private final double cruiseTime;
    private final double cruiseDistance;
    private final double distance;
    private final double startingVelocity;

    public MotionProfileAccelTrig(double maxAccel, double maxVelocity, double distance, double startingVelocity) {
        this.startingVelocity = startingVelocity;
        double rampUpDistance = calcRampDistance(maxAccel, startingVelocity, maxVelocity);
        double rampDownDistance = calcRampDistance(maxAccel, maxVelocity, 0.0);
        if (distance < rampUpDistance + rampDownDistance) {
            maxVelocity = Math.sqrt(maxAccel * distance + startingVelocity * startingVelocity) / Math.sqrt(2);
        }
        rampUpDistance = calcRampDistance(maxAccel, startingVelocity, maxVelocity);
        rampDownDistance = calcRampDistance(maxAccel, maxVelocity, 0.0);
        this.rampUpDistance = rampUpDistance;
        this.rampDownDistance = rampDownDistance;
        this.rampUpTime = calcRampTime(maxAccel, maxVelocity - startingVelocity);
        this.rampDownTime = calcRampTime(maxAccel, maxVelocity);
        this.maxAccel = maxAccel;
        this.maxVelocity = maxVelocity;
        this.cruiseDistance = distance - this.rampUpDistance - this.rampDownDistance;
        this.cruiseTime = this.cruiseDistance / maxVelocity;
        this.distance = distance;
    }

    private static double calcRampDistance(double glimit, double startingVelocity, double endingVelocity) {
        double deltav = endingVelocity - startingVelocity;
        return ((deltav * deltav) / glimit + Math.min(startingVelocity, endingVelocity) * calcRampTime(glimit, deltav));
    }

    private static double calcRampTime(double glimit, double deltav) {
        return (2.0 * Math.abs(deltav)) / glimit;
    }

    private static double trigRamp2ndIntegral(double x, double g, double s) {
        return (g / 4.0) * (x * x + (2.0 * s * s * Math.cos((Math.PI * g * x) / s)) / (g * g * Math.PI * Math.PI)) - (s * s) / (2.0 * g * Math.PI * Math.PI);
    }

    private static double trigRamp1stIntegral(double x, double g, double s) {
        double pigx = Math.PI * g * x;
        return (pigx - s * Math.sin(pigx / s)) / (2.0 * Math.PI);
    }

    private static double trigRamp(double x, double g, double s) {
        return (g - g * Math.cos((Math.PI * g * x) / s)) / 2.0;
    }

    private double rangeResult(double time, double middleValue, BiFunction<Double, Double, Double> callback, double endState) {
        if (time < rampUpTime) {
            return callback.apply(time, maxVelocity - startingVelocity);
        } else if (time >= rampUpTime && time <= cruiseTime + rampUpTime) {
            return middleValue;
        } else if (time > cruiseTime + rampUpTime && time < cruiseTime + rampUpTime + rampDownTime) {
            return (-callback.apply(time - rampUpTime - cruiseTime - rampDownTime, maxVelocity) + endState);
        } else {
            return endState;
        }
    }

    @Override
    public double totalProfileTime() {
        return rampUpTime + cruiseTime + rampDownTime;
    }

    public double getRampUpDistance() {
        return rampUpDistance;
    }

    public double getRampDownDistance() {
        return rampDownDistance;
    }

    public double getCruiseDistance() {
        return cruiseDistance;
    }

    @Override
    public double profilePosition(double time) {
        return rangeResult(time, maxVelocity * (time - rampUpTime) + rampUpDistance, (x, deltav) -> trigRamp2ndIntegral(x, maxAccel, deltav) + (maxVelocity - deltav) * x, distance);
    }

    @Override
    public double profileVelocity(double time) {
        return rangeResult(time, maxVelocity, (x, deltav) -> trigRamp1stIntegral(x, maxAccel, deltav) + (maxVelocity - deltav), 0);
    }

    @Override
    public double profileAccel(double time) {
        return rangeResult(time, 0.0, (x, deltav) -> trigRamp(x, maxAccel, deltav), 0);
    }
}
