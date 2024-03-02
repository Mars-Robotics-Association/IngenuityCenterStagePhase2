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
public final class BackStageRedCenterPath extends AutoPath {
    public static double pushX;
    public static double pushY;
    public static double pushAngle;
    public static double invPushAngle;
    public static double deliveryX;
    public static double preDeliveryX;
    public static double deliveryY;

    // Constructor - Instantiate this class before waitForStart ==============
    public BackStageRedCenterPath(OpMode newOpMode, PhaseTwoBot newBot, MecanumDrive newDrive) {
        super(StartingPosition.BACK_RED, newOpMode, newBot, newDrive);

        pushX = initX;
        pushY = initY + 19;
        pushAngle = initAngle;
        invPushAngle = AutoPath.reverseAngle(pushAngle);
        deliveryX = 48;
        preDeliveryX = deliveryX - 6.5;
        deliveryY = -35;
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
                .splineTo(new Vector2d(11, -33), Math.toRadians(invPushAngle), mediumSpeed)
                .splineTo(new Vector2d(26, -42), Math.toRadians(0), mediumSpeed)
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
