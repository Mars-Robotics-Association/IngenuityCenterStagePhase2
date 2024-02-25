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
    public static double initX = 11;
    public static double initY = -61;
    public static double initAngle = 90;
    public static double pushX = initX;
    public static double pushY = initY + 35.5;
    public static double pushAngle = initAngle;
    public static double invPushAngle = ((int) pushAngle + 180) % 360;
    public static double deliveryX = 48;
    public static double preDeliveryX = deliveryX - 6.5;
    public static double deliveryY = -35;
    public static double parkingX = 58;
    public static double parkingY = -56;
    public static int backDelivery = Math.min(PhaseTwoBot.armMax, 2110);

    public PhaseTwoBot bot ;
    public OpMode opMode ;
    MecanumDrive drive ;

    // Constructor - Instantiate this class before waitForStart ==============
    public BackStageRedCenterPath(OpMode newOpMode, PhaseTwoBot newBot, MecanumDrive newDrive) {
        opMode = newOpMode ;
        bot = newBot ;
        drive = newDrive ;
    }

    // Run this after Start ==================================================
    @Override
    public void runAutoPath() {
        TranslationalVelConstraint slow = new TranslationalVelConstraint(15);
        TranslationalVelConstraint mediumSpeed = new TranslationalVelConstraint(25);

        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .afterTime(0.0, bot.gripperArm().moveArmToStopAction(1, true))
                .splineTo(new Vector2d(pushX, pushY), Math.toRadians(pushAngle), mediumSpeed)
                .stopAndAdd(new SequentialAction(bot.gripperArm().gripperHalfOpenAction(),
                        bot.gripperArm().moveArmToStopAction(1, true)))
                .setReversed(true)
                .afterTime(0.0, bot.gripperArm().moveArmToPositionAction(backDelivery, "start moving", true))
                .splineTo(new Vector2d(11, -33), Math.toRadians(invPushAngle), mediumSpeed)
                .splineTo(new Vector2d(26, -42), Math.toRadians(0), mediumSpeed)
                .splineTo(new Vector2d(preDeliveryX, deliveryY), Math.toRadians(0), mediumSpeed)
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
