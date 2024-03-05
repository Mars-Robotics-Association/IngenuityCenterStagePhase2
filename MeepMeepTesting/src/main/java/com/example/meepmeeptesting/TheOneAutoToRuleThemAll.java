package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

enum Alliance {
    BLUE,
    RED
}

enum PropPosition {
    LEFT, MIDDLE, RIGHT
}

enum StagePosition {
    FRONT, BACK
}

enum PurplePlacement {
    FRONT, MIDDLE, BACK
}

public class TheOneAutoToRuleThemAll {
    private final Alliance alliance;
    private final PropPosition propPosition;
    private final StagePosition stagePosition;


    public static final double directionAudience = 180;
    public static final double directionBackdrop = 0;
    public static final double directionOpponent = 270;
    public static final double directionSelf = 90;

    public final double initX;
    public final double initY;
    public final double initAngle;
    public double centerLaneY = 10;
    public double relTurn = -14;
    public double deliveryX = 48;
    public double preDeliveryX = deliveryX - 6.5;
    public double parkingX = 58;
    public final double parkingY;


    public TheOneAutoToRuleThemAll(Alliance alliance, PropPosition propPosition, StagePosition stagePosition) {
        this.alliance = alliance;
        this.propPosition = alliance == Alliance.BLUE ? propPosition :
                propPosition == PropPosition.LEFT ? PropPosition.RIGHT :
                        propPosition == PropPosition.RIGHT ? PropPosition.LEFT :
                                PropPosition.MIDDLE;
        this.stagePosition = stagePosition;

        initX = stagePosition == StagePosition.BACK ? 12 : -36;
        initY = 61;
        initAngle = 270;
        parkingY = stagePosition == StagePosition.BACK ? 56 : 8;
    }

    public static double reverseAngle(double degrees) {
        return (degrees + 180.0) % 360.0;
    }

    public static double mirrorAngle(double degrees) {
        return ((360.0) - degrees) % 360.0;
    }

    private Vector2d relCoords(double xOffset, double yOffset) {
        if (alliance == Alliance.BLUE) {
            return new Vector2d(initX + xOffset, initY + yOffset);
        } else {
            return new Vector2d(initX + xOffset, 0.0 - initY - yOffset);
        }
    }

    private Vector2d absCoords(double x, double y) {
        return new Vector2d(x, alliance == Alliance.BLUE ? y : 0.0 - y);
    }

    private double absHeading(double degrees) {
        return Math.toRadians(alliance == Alliance.BLUE ? degrees : mirrorAngle(degrees));
    }

    private double relHeading(double degreesOffset) {
        return Math.toRadians((alliance == Alliance.BLUE ? initAngle : mirrorAngle(initAngle)) + (alliance == Alliance.BLUE ? degreesOffset : 0.0 - degreesOffset));
    }

    public TrajectorySequenceBuilder main(TrajectorySequenceBuilder trajBuilder) {
        trajBuilder = trajBuilder
                .splineTo(relCoords(0, -8), relHeading(0))
                .waitSeconds(1);
        trajBuilder = placePurplePixel(trajBuilder);
        if (stagePosition == StagePosition.FRONT) {
            trajBuilder = driveFromFrontToBack(trajBuilder);
        } else {
            trajBuilder = driveFromBackToBack(trajBuilder);
        }
        trajBuilder = placeYellowPixel(trajBuilder);
        trajBuilder = park(trajBuilder);
        return trajBuilder;
    }

    public Pose2d getStartingPose() {
        return new Pose2d(relCoords(0, 0), relHeading(0));
    }

    private TrajectorySequenceBuilder driveFromFrontToBack(TrajectorySequenceBuilder trajBuilder) {
        return trajBuilder
                .splineTo(relCoords(-11, relTurn + 10), absHeading(directionAudience))
                .splineTo(relCoords(-22, relTurn), relHeading(0))
                .splineTo(relCoords(-22, -33), relHeading(0))
                .splineTo(absCoords(initX, centerLaneY), absHeading(directionBackdrop))
                .splineTo(absCoords(18, centerLaneY), absHeading(directionBackdrop));
    }

    private TrajectorySequenceBuilder driveFromBackToBack(TrajectorySequenceBuilder trajBuilder) {
        return trajBuilder
                .splineTo(absCoords(23, 58), absHeading(directionBackdrop));
    }

    private TrajectorySequenceBuilder placePurplePixel(TrajectorySequenceBuilder trajBuilder) {
        switch (propPosition) {
            case MIDDLE:
                trajBuilder = trajBuilder.splineTo(relCoords(+2, -29), relHeading(20));
                break;
            case RIGHT:
                trajBuilder = trajBuilder.splineTo(relCoords(-5.5, -23), relHeading(-40));
                break;
            default:
                trajBuilder = trajBuilder.splineTo(relCoords(10, -23), relHeading(30));
                break;
        }
        trajBuilder = trajBuilder
//                .afterTime(0, new SequentialAction(bot.gripperArm().gripperHalfOpenAction()))
                .setReversed(true)
                .splineTo(relCoords(0, relTurn), absHeading(reverseAngle(initAngle)));
        return trajBuilder;
    }

    private TrajectorySequenceBuilder placeYellowPixel(TrajectorySequenceBuilder trajBuilder) {
        double deliveryY = propPosition == PropPosition.MIDDLE ? 35 :
                propPosition == PropPosition.RIGHT ? 27 : 41;

        return trajBuilder

                .splineTo(absCoords(preDeliveryX, deliveryY), absHeading(directionBackdrop))
//                .afterTime(0.15, bot.gripperArm().moveArmToPositionAction(backDelivery, "start moving", true))
                .splineTo(absCoords(deliveryX, deliveryY), absHeading(directionBackdrop))
                .waitSeconds(1.5)
//                .stopAndAdd(new SequentialAction(
//                        new TimeoutAction(bot.gripperArm().moveArmToPositionAction(backDelivery, "finish moving", true), 2.5),
//                        bot.gripperArm().gripperOpenAction(),
//                        new SleepAction(0.5),
//                        bot.gripperArm().setWristTuckedUp(),
//                        bot.gripperArm().moveArmToStopAction(1, false)))
                .setReversed(false)
//                .afterTime(0.0, new SequentialAction(
//                        bot.gripperArm().moveArmToStopAction(0),
//                        bot.gripperArm().lowerArmToLimit()
//                ))
                .splineTo(absCoords(preDeliveryX, deliveryY), absHeading(directionAudience));
    }

    private TrajectorySequenceBuilder park(TrajectorySequenceBuilder trajBuilder) {
        return trajBuilder
                .strafeTo(absCoords(preDeliveryX, parkingY))
                .setReversed(true)
                .splineTo(absCoords(parkingX, parkingY), absHeading(directionBackdrop));
    }

}
