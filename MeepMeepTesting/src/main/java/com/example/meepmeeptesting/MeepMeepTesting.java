package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double initX = -36;
    public static double initY = 61;
    public static double initAngle = 270;
    public static double pushX = initX - 10;
    public static double pushY = initY - 26;
    public static double pushAngle = initAngle - 40;
    public static double invPushAngle = ((int) pushAngle + 180) % 360;
    public static double backoffDistance = 4;
    public static double backOffXby = Math.cos(Math.toRadians(invPushAngle)) * backoffDistance;
    public static double backOffYby = Math.sin(Math.toRadians(invPushAngle)) * backoffDistance;
    public static double deliveryX = 48;
    public static double preDeliveryX = deliveryX - 6.5;
    public static double deliveryY = 27;
    public static double parkingX = 58;
    public static double parkingY = 10;
    public static int backDelivery = Math.min(PhaseTwoBot.armMax, 2110);


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(initX, initY, Math.toRadians(initAngle)))

//                        .afterTime(0.0, bot.gripperArm().moveArmToStopAction(1, true))
                        .splineTo(new Vector2d(pushX, pushY), Math.toRadians(pushAngle))
                        .setReversed(true)
//                        .afterTime(5.0, bot.gripperArm().moveArmToPositionAction(backDelivery, "start moving", true))
                        .splineTo(new Vector2d(pushX + backOffXby, pushY + backOffYby), Math.toRadians(invPushAngle))
                        .splineTo(new Vector2d(-49, 49), Math.toRadians(180))
                        .splineTo(new Vector2d(-55, pushY - 5), Math.toRadians(270))
                        .splineTo(new Vector2d(-36, 11), Math.toRadians(0))
                        .splineTo(new Vector2d(18, 11), Math.toRadians(0))
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
