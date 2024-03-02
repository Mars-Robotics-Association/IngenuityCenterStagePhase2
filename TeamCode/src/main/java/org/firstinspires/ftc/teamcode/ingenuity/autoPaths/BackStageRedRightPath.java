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
public final class BackStageRedRightPath extends AutoPath {
    public static double pushX;
    public static double pushY;
    public static double pushAngle;
    public static double invPushAngle;
    public static double backoffDistance;
    public static double backOffXby;
    public static double backOffYby;
    public static double deliveryX;
    public static double preDeliveryX;
    public static double deliveryY;

    // Constructor - Instantiate this class before waitForStart ==============
    public BackStageRedRightPath(OpMode newOpMode, PhaseTwoBot newBot, MecanumDrive newDrive) {
        super(StartingPosition.BACK_RED, newOpMode, newBot, newDrive);
        pushX = initX + 8;
        pushY = initY + 24;
        pushAngle = initAngle - 30;
        invPushAngle = AutoPath.reverseAngle(pushAngle);
        backoffDistance = 4;
        backOffXby = Math.sin(Math.toRadians(invPushAngle - 90)) * backoffDistance;
        backOffYby = Math.cos(Math.toRadians(invPushAngle - 90)) * backoffDistance;
        deliveryX = 48;
        preDeliveryX = deliveryX - 6.5;
        deliveryY = -41;
    }

    // Run this after Start ==================================================
    @Override
    public void runAutoPath() {
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .stopAndAdd(new SleepAction(initPause))
                .afterTime(0.0, bot.gripperArm().moveArmToStopAction(1, true))
                .splineTo(new Vector2d(pushX, pushY), Math.toRadians(pushAngle), mediumSpeed)
                .stopAndAdd(new SequentialAction(bot.gripperArm().gripperHalfOpenAction(),
                        bot.gripperArm().moveArmToStopAction(1, true)))
                .setReversed(true)
                .splineTo(new Vector2d(pushX - backOffXby, pushY + backOffYby), Math.toRadians(invPushAngle), mediumSpeed)
                .splineTo(new Vector2d(26, -49), Math.toRadians(0), mediumSpeed)
                .splineTo(new Vector2d(preDeliveryX, deliveryY), Math.toRadians(0), mediumSpeed)
                .afterTime(armDelay, bot.gripperArm().moveArmToPositionAction(backDelivery, "start moving", true))
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
