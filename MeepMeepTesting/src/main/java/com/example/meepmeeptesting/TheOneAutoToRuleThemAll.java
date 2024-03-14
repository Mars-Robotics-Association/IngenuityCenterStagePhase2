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
    private final StagePosition stagePosition;


    public static final double directionAudience = 180;
    public static final double directionBackdrop = 0;
    public static final double directionOpponent = 270;
    public static final double directionSelf = 90;

    public static double purpleXMiddle = 1.5;
    public static double purpleXRight = -1.5;
    public static double purpleXLeft = 2;
    public static double purpleYSide = -19;
    public static double purpleYMiddle = -29;
    public static double purpleAngleLeft = 50;
    public static double purpleAngleMiddle = 15;
    public static double purpleAngleRight = -50;


    public static double yellowLeft = 40;
    public static double yellowMiddle = 34;
    public static double yellowRight = 26;

    private final double initX;
    private final double initY;
    private final double initAngle;
    private final double parkingY;

    public static double initXFront = -36;
    public static double initXBack = 12;
    public static double centerLaneY = 10;
    public static double relTurn = -14;
    public static double deliveryX = 46;
    public static double preDeliveryX = deliveryX - 4;
    public static double parkingX = 58;
    public static double parkingYFront = 8;
    public static double parkingYBack = 56;
    private PropPosition propPosition;

    public TheOneAutoToRuleThemAll(Alliance alliance, PropPosition propPosition, StagePosition stagePosition) {
        this.alliance = alliance;
        this.propPosition = propPosition;
        this.stagePosition = stagePosition;

        initX = stagePosition == StagePosition.BACK ? initXBack : initXFront;
        initY = 61;
        initAngle = 270;
        parkingY = stagePosition == StagePosition.BACK ? parkingYBack : parkingYFront;
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

    public TrajectorySequenceBuilder start(TrajectorySequenceBuilder trajBuilder) {
        trajBuilder = trajBuilder
                .splineTo(relCoords(0, -12), relHeading(0))
                .waitSeconds(1);
        return afterScan(trajBuilder);
    }

    public TrajectorySequenceBuilder afterScan(TrajectorySequenceBuilder trajBuilder) {
        // reverse left and right if we're on the red alliance
        propPosition = alliance == Alliance.BLUE ? propPosition :
                propPosition == PropPosition.LEFT ? PropPosition.RIGHT :
                        propPosition == PropPosition.RIGHT ? PropPosition.LEFT :
                                PropPosition.MIDDLE;

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
                .waitSeconds(1)
                .splineTo(relCoords(-11, relTurn + 10), absHeading(directionAudience))
                .splineTo(relCoords(-22, relTurn), relHeading(0))
                .splineTo(relCoords(-22, -33), relHeading(0))
                .splineTo(absCoords(initX, centerLaneY), absHeading(directionBackdrop))
                .splineTo(absCoords(22, centerLaneY), absHeading(directionBackdrop));
    }

    private TrajectorySequenceBuilder driveFromBackToBack(TrajectorySequenceBuilder trajBuilder) {
        return trajBuilder
                .splineTo(absCoords(23, 56), absHeading(directionBackdrop));
    }

    private TrajectorySequenceBuilder placePurplePixel(TrajectorySequenceBuilder trajBuilder) {
        switch (propPosition) {
            case MIDDLE:
                trajBuilder = trajBuilder.splineTo(relCoords(purpleXMiddle, purpleYMiddle), relHeading(purpleAngleMiddle));
                break;
            case RIGHT:
                trajBuilder = trajBuilder.splineTo(relCoords(purpleXRight, purpleYSide), relHeading(purpleAngleRight));
                break;
            default:
                trajBuilder = trajBuilder.splineTo(relCoords(purpleXLeft, purpleYSide), relHeading(purpleAngleLeft));
                break;
        }
        trajBuilder = trajBuilder
//                .afterTime(0, new SequentialAction(bot.gripperArm().gripperHalfOpenAction()))
                .waitSeconds(0.3)
                .setReversed(true)
                .splineTo(relCoords(0, -12), absHeading(reverseAngle(initAngle)));
        return trajBuilder;
    }

    private TrajectorySequenceBuilder placeYellowPixel(TrajectorySequenceBuilder trajBuilder) {
        double deliveryY = propPosition == PropPosition.MIDDLE ? yellowMiddle :
                propPosition == PropPosition.RIGHT ? yellowRight : yellowLeft;

        return trajBuilder

                .splineTo(absCoords(preDeliveryX, deliveryY), absHeading(directionBackdrop)) // line up
//                .afterTime(0.15, bot.gripperArm().moveArmToPositionAction(backDelivery, "start moving", true))
                .splineTo(absCoords(deliveryX, deliveryY), absHeading(directionBackdrop)) // final approach
                .waitSeconds(1.5)
//                .stopAndAdd(new SequentialAction(
//                        new TimeoutAction(bot.gripperArm().moveArmToPositionAction(backDelivery, "finish moving", true), 2.5),
//                        bot.gripperArm().gripperOpenAction(),
//                        new SleepAction(0.25),
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
