package org.firstinspires.ftc.teamcode;

public abstract class MotionProfile {
    public abstract double totalProfileTime();

    public boolean insideProfile(double time) {
        return time >= 0 && time < totalProfileTime();
    }

    public abstract double profilePosition(double time);

    public abstract double profileVelocity(double time);

    public abstract double profileAccel(double time);
}


