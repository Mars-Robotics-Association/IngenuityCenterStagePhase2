package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class WormGroup extends WormMotor implements Iterable<WormMotor> {
    public static double armFollowerKp = 0.000;
    public static double armFollowerKi = 0.005;
    public static int armFollowerErrorThreshold = 0;
    public static int armFollowerErrorSumThreshold = 10;
    public static double armFollowerErrorSumDecay = 0.65;

    private final WormMotor[] group;

    private final int[] motorFollowerErrorSum;
    private final Telemetry telemetry;


    public WormGroup(Telemetry telemetry, @NonNull WormMotor leader, WormMotor... followers) {
        group = new WormMotor[followers.length + 1];
        group[0] = leader;
        System.arraycopy(followers, 0, group, 1, followers.length);
        motorFollowerErrorSum = new int[followers.length + 1];
        this.telemetry = telemetry;
    }

    @Override
    public void set(double speed) {
        group[0].set(speed);

        for (int i = 1; i < group.length; i++) {
            int error = group[0].getCurrentPosition() - group[i].getCurrentPosition();

            motorFollowerErrorSum[i] = (int) (motorFollowerErrorSum[i] * armFollowerErrorSumDecay) + error;
            double windupLimit = 0.25 / armFollowerKi;
            if (motorFollowerErrorSum[i] > windupLimit) {
                motorFollowerErrorSum[i] = (int) windupLimit;
            } else if (motorFollowerErrorSum[i] < -windupLimit) {
                motorFollowerErrorSum[i] = (int) -windupLimit;
            }
            if (Math.abs(motorFollowerErrorSum[i]) < armFollowerErrorSumThreshold) {
                motorFollowerErrorSum[i] = 0;
            }

            telemetry.addData("arm motor error " + i, error);
            telemetry.addData("motor err int" + i, motorFollowerErrorSum);

            if (Math.abs(error) < armFollowerErrorThreshold) {
                error = 0;
            }

            double armPower = group[0].get();

            group[i].set(armPower + armFollowerKp * error + armFollowerKi * motorFollowerErrorSum[i]);
        }
    }

    @Override
    public double get() {
        return group[0].get();
    }

    /**
     * @return All motor target speeds as a percentage of output
     */
    public List<Double> getSpeeds() {
        return Arrays.stream(group)
                .map(WormMotor::get)
                .collect(Collectors.toList());
    }

    @Override
    public double getVelocity() {
        return group[0].getCorrectedVelocity();
    }

    @Override
    public double getPositionCoefficient() {
        return group[0].getPositionCoefficient();
    }

    @Override
    public double getPositionIntegralCoefficient() {
        return group[0].getPositionIntegralCoefficient();
    }

    /**
     * @return All current velocities of the motors in the group in units of distance
     * per second which is by default ticks / second
     */
    public List<Double> getVelocities() {
        return Arrays.stream(group)
                .map(WormMotor::getRate)
                .collect(Collectors.toList());
    }

    @NonNull
    @Override
    public Iterator<WormMotor> iterator() {
        return Arrays.asList(group).iterator();
    }

    @Override
    public Encoder setDistancePerPulse(double distancePerPulse) {
        Encoder leaderEncoder = group[0].setDistancePerPulse(distancePerPulse);
        for (int i = 1; i < group.length; i++) {
            group[i].setDistancePerPulse(distancePerPulse);
        }
        return leaderEncoder;
    }

    /**
     * @return The position of every motor in the group in units of distance
     * which is by default ticks
     */
    public List<Double> getPositions() {
        return Arrays.stream(group)
                .map(WormMotor::getDistance)
                .collect(Collectors.toList());
    }

    @Override
    public void setRunMode(RunMode runmode) {
        group[0].setRunMode(runmode);
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        for (WormMotor motor : group) {
            motor.setZeroPowerBehavior(behavior);
        }
    }

    @Override
    public void resetEncoder() {
        group[0].resetEncoder();
    }

    @Override
    public void stopAndResetEncoder() {
        group[0].stopAndResetEncoder();
    }

    @Override
    public void setPositionCoefficient(double kp) {
        group[0].setPositionCoefficient(kp);
    }

    @Override
    public void setPositionIntegralCoefficient(double ki) {
        group[0].setPositionIntegralCoefficient(ki);
    }

    @Override
    public void setPositionPI(double kp, double ki) {
        group[0].setPositionPI(kp, ki);
    }

    @Override
    public boolean atTargetPosition() {
        return group[0].atTargetPosition();
    }

    @Override
    public void setTargetPosition(int target) {
        group[0].setTargetPosition(target);
    }

    @Override
    public void setTargetDistance(double target) {
        group[0].setTargetDistance(target);
    }

    @Override
    public void setPositionTolerance(double tolerance) {
        group[0].setPositionTolerance(tolerance);
    }

    @Override
    public void setVeloCoefficients(double kp, double ki, double kd) {
        group[0].setVeloCoefficients(kp, ki, kd);
    }

    @Override
    public void setFeedforwardCoefficients(double ks, double kv) {
        group[0].setFeedforwardCoefficients(ks, kv);
    }

    @Override
    public void setFeedforwardCoefficients(double ks, double kv, double ka) {
        group[0].setFeedforwardCoefficients(ks, kv, ka);
    }

    /**
     * @return true if the motor group is inverted
     */
    @Override
    public boolean getInverted() {
        return group[0].getInverted();
    }

    /**
     * Set the motor group to the inverted direction or forward direction.
     * This directly affects the speed rather than the direction.
     *
     * @param isInverted The state of inversion true is inverted.
     */
    @Override
    public void setInverted(boolean isInverted) {
        for (WormMotor motor : group) {
            motor.setInverted(isInverted);
        }
    }

    /**
     * Disables all the motor devices.
     */
    @Override
    public void disable() {
        for (WormMotor x : group) {
            x.disable();
        }
    }

    @Override
    public void setTelemetry(TelemetryPacket packet, String stepName) {
        group[0].setTelemetry(packet, stepName);
    }

    /**
     * @return a string characterizing the device type
     */
    @Override
    public String getDeviceType() {
        return "Motor Group";
    }

    /**
     * Stops all motors in the group.
     */
    @Override
    public void stopMotor() {
        for (WormMotor x : group) {
            x.stopMotor();
        }
    }

}
