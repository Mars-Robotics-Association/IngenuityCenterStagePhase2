package org.firstinspires.ftc.teamcode;

public class MotionProfile {
//    private final double maxAccel;
//    private final double maxVelocity;
//    private final double accelerationDt;
//    public MotionProfile(double maxAccel, double maxVelocity) {
//        this.maxAccel = maxAccel;
//        this.maxVelocity = maxVelocity;
//
//        // Calculate the time it takes to accelerate to max velocity
//        double accelerationDt = maxVelocity / maxAccel;
//
//        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
//        double halfwayDistance = distance / 2
//        acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
//    }
//
//    double motion_profile(max_acceleration, max_velocity, distance, elapsed_time) {
//
//
//        // Calculate the time it takes to accelerate to max velocity
//        acceleration_dt = max_velocity / max_acceleration
//
//        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
//        halfway_distance = distance / 2
//        acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
//
//        if (acceleration_distance > halfway_distance) {
//            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration))
//        }
//
//        acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
//
//        // recalculate max velocity based on the time we have to accelerate and decelerate
//        max_velocity = max_acceleration * acceleration_dt
//
//        // we decelerate at the same rate as we accelerate
//        deceleration_dt = acceleration_dt
//
//        // calculate the time that we're at max velocity
//        cruise_distance = distance - 2 * acceleration_distance
//        cruise_dt = cruise_distance / max_velocity
//        deceleration_time = acceleration_dt + cruise_dt
//
//        // check if we're still in the motion profile
//        entire_dt = acceleration_dt + cruise_dt + deceleration_dt
//        if (elapsed_time > entire_dt) {
//            return distance
//        }
//
//        // if we're accelerating
//        if (elapsed_time < acceleration_dt) {
//            // use the kinematic equation for acceleration
//            return 0.5 * max_acceleration * elapsed_time ** 2
//        }
//
//        // if we're cruising
//        else if (elapsed_time < deceleration_time) {
//            acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
//            cruise_current_dt = elapsed_time - acceleration_dt
//
//            // use the kinematic equation for constant velocity
//            return acceleration_distance + max_velocity * cruise_current_dt
//        }
//
//        // if we're decelerating
//        else {
//            acceleration_distance = 0.5 * max_acceleration * acceleration_dt ** 2
//            cruise_distance = max_velocity * cruise_dt
//            deceleration_time = elapsed_time - deceleration_time
//
//            // use the kinematic equations to calculate the instantaneous desired position
//            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * deceleration_time ** 2
//        }
//    }
}
