package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double initX = 12;
    public static double initY = 61;
    public static double initAngle = 270;
    public static double pushX = initX;
    public static double pushY = initY - 33.5;
    public static double pushAngle = initAngle;
    public static double invPushAngle = ((int) pushAngle + 180) % 360;
    public static double deliveryX = 48;
    public static double preDeliveryX = deliveryX - 6.5;
    public static double deliveryY = 35;
    public static double parkingX = 58;
    public static double parkingY = 56;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(initX, initY, Math.toRadians(initAngle)))

                                .splineTo(new Vector2d(pushX+3, pushY), Math.toRadians(pushAngle+20))
                                .setReversed(true)
                                .splineTo(new Vector2d(initX, initY-8), Math.toRadians(initAngle-180))
                                .setReversed(false)
                        .splineTo(new Vector2d(pushX, pushY), Math.toRadians(pushAngle))
//                        .stopAndAdd(new SequentialAction(bot.gripperArm().gripperHalfOpenAction(),
//                                bot.gripperArm().moveArmToStopAction(1, true)))
                        .setReversed(true)
//                        .afterTime(0.5, bot.gripperArm().moveArmToPositionAction(backDelivery, "start moving", true))
                       // .splineTo(new Vector2d(11, 33), Math.toRadians(invPushAngle))
                        .splineTo(new Vector2d(26, 42), Math.toRadians(0))
                        .splineTo(new Vector2d(preDeliveryX, deliveryY), Math.toRadians(0))
                        .splineTo(new Vector2d(deliveryX, deliveryY), Math.toRadians(0))
//                        .stopAndAdd(new SequentialAction(
//                                new TimeoutAction(bot.gripperArm().moveArmToPositionAction(backDelivery, "finish moving", true), 2.5),
//                                bot.gripperArm().gripperOpenAction(),
//                                new SleepAction(0.5),
//                                bot.gripperArm().setWristTuckedUp(),
//                                bot.gripperArm().moveArmToStopAction(1, false)))
                        .setReversed(false)
//                        .afterTime(0.0, new SequentialAction(
//                                bot.gripperArm().moveArmToStopAction(0),
//                                bot.gripperArm().lowerArmToLimit()
//                        ))
                        .splineTo(new Vector2d(preDeliveryX, deliveryY), Math.toRadians(180))
                        .strafeTo(new Vector2d(preDeliveryX, parkingY))
                        .setReversed(true)
                        .splineTo(new Vector2d(parkingX, parkingY), Math.toRadians(0))


                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
