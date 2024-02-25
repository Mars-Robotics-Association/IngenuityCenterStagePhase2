package org.firstinspires.ftc.teamcode.ingenuity.autoPaths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PhaseTwoBot;
import org.firstinspires.ftc.teamcode.TimeoutAction;

@Config
public final class FrontStageBlueRightPath extends AutoPath {
    public static double initPause = 5.0;
    public static double initX = -36;
    public static double initY = 61;
    public static double initAngle = 270;
    public static double pushX = initX - 6;
    public static double pushY = initY - 26;
    public static double pushAngle = initAngle - 40;
    public static double invPushAngle = ((int) pushAngle + 180) % 360;
    public static double backoffDistance = 4;
    public static double backOffXby = Math.cos(Math.toRadians(invPushAngle)) * backoffDistance;
    public static double backOffYby = Math.sin(Math.toRadians(invPushAngle)) * backoffDistance;
    public static double centerLaneY = 11;
    public static double deliveryX = 48;
    public static double preDeliveryX = deliveryX - 6.5;
    public static double deliveryY = 27;
    public static double parkingX = 58;
    public static double parkingY = 8;
    public static int backDelivery = Math.min(PhaseTwoBot.armMax, 2110);
    public static double armDelay = 4.25;

    public PhaseTwoBot bot ;
    public OpMode opMode ;
    MecanumDrive drive ;

    // Constructor - Instantiate this class before waitForStart ==============
    public FrontStageBlueRightPath(OpMode newOpMode, PhaseTwoBot newBot, MecanumDrive newDrive) {
        opMode = newOpMode ;
        bot = newBot ;
        drive = newDrive ;
    }

    // Run this after Start ==================================================
    @Override
    public void runAutoPath() {
        TranslationalVelConstraint slow = new TranslationalVelConstraint(15);

        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .stopAndAdd(new SleepAction(initPause))
                .afterTime(0.0, bot.gripperArm().moveArmToStopAction(1, true))
                .splineTo(new Vector2d(pushX, pushY), Math.toRadians(pushAngle))
                .afterTime(0.85, bot.gripperArm().moveArmToPositionAction(PhaseTwoBot.armDropOne))
                .setReversed(true)
                .afterTime(armDelay, bot.gripperArm().moveArmToPositionAction(backDelivery, "start moving", true))
                .splineTo(new Vector2d(pushX + backOffXby, pushY + backOffYby), Math.toRadians(invPushAngle))
                .splineTo(new Vector2d(-49, 49), Math.toRadians(180))
                .splineTo(new Vector2d(-55, pushY - 5), Math.toRadians(initAngle))
                .splineTo(new Vector2d(-36, centerLaneY), Math.toRadians(0))
                .splineTo(new Vector2d(18, centerLaneY), Math.toRadians(0))
                .splineTo(new Vector2d(preDeliveryX, deliveryY), Math.toRadians(0))
                .splineTo(new Vector2d(deliveryX, deliveryY), Math.toRadians(0), slow)
                .stopAndAdd(new SequentialAction(
                        new TimeoutAction(bot.gripperArm().moveArmToPositionAction(backDelivery, "finish moving", true), 2.5),
                        bot.gripperArm().gripperOpenAction(),
                        new SleepAction(0.5),
                        bot.gripperArm().setWristTuckedUp(),
                        bot.gripperArm().moveArmToStopAction(1, false)))
                .setReversed(false)
                .afterTime(0.0, new SequentialAction(
                        bot.gripperArm().moveArmToStopAction(0),
                        bot.gripperArm().lowerArmToLimit()
                ))
                .splineTo(new Vector2d(preDeliveryX, deliveryY), Math.toRadians(180))
                .strafeTo(new Vector2d(preDeliveryX, parkingY))
                .setReversed(true)
                .splineTo(new Vector2d(parkingX, parkingY), Math.toRadians(0))
                .build());
    }
}
