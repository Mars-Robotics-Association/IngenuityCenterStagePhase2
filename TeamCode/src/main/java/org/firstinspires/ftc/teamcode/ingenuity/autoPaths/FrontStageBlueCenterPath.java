package org.firstinspires.ftc.teamcode.ingenuity.autoPaths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PhaseTwoBot;
import org.firstinspires.ftc.teamcode.TimeoutAction;

@Config
public final class FrontStageBlueCenterPath extends AutoPath {
    public static double pushX = initX;
    public static double pushY = initY - 29;
    public static double pushAngle = initAngle;
    public static double invPushAngle = AutoPath.reverseAngle(pushAngle);
    public static double centerLaneY = 10;
    public static double deliveryX = 48;
    public static double preDeliveryX = deliveryX - 6.5;
    public static double deliveryY = 31;

    // Constructor - Instantiate this class before waitForStart ==============
    public FrontStageBlueCenterPath(OpMode newOpMode, PhaseTwoBot newBot, MecanumDrive newDrive) {
        super(StartingPosition.FRONT_BLUE, newOpMode, newBot, newDrive);
    }

    // Run this after Start ==================================================
    @Override
    public void runAutoPath() {
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .stopAndAdd(new SleepAction(initPause))
                .afterTime(0.0, bot.gripperArm().moveArmToStopAction(1, true))
                .splineTo(new Vector2d(pushX, pushY), Math.toRadians(pushAngle), mediumSpeed)
                .afterTime(0.85, bot.gripperArm().moveArmToPositionAction(PhaseTwoBot.armDropOne))
                .setReversed(true)
                .splineTo(new Vector2d(pushX, pushY - 8), Math.toRadians(invPushAngle), mediumSpeed)
                .splineTo(new Vector2d(-48, -47), Math.toRadians(180), mediumSpeed)
                .splineTo(new Vector2d(-55, pushY - 8), Math.toRadians(initAngle), mediumSpeed)
                .splineTo(new Vector2d(-36, centerLaneY), Math.toRadians(0), mediumSpeed)
                .splineTo(new Vector2d(18, centerLaneY), Math.toRadians(0), mediumSpeed)
                .splineTo(new Vector2d(preDeliveryX, deliveryY), Math.toRadians(0), slow)
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
