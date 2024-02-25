package org.firstinspires.ftc.teamcode.ingenuity.autoPaths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PhaseTwoBot;
import org.firstinspires.ftc.teamcode.PropDetection;
import org.firstinspires.ftc.teamcode.TimeoutAction;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config
public final class BackStageBlueCenterPath extends AutoPath {
    public static double initX = 12;
    public static double initY = 61;
    public static double initAngle = 270;
    public static double pushX = initX + 2.5;
    public static double pushY = initY - 29;
    public static double pushAngle = initAngle + 20;
    public static double invPushAngle = ((int) pushAngle + 180) % 360;
    public static double deliveryX = 48;
    public static double preDeliveryX = deliveryX - 6.5;
    public static double deliveryY = 35;
    public static double parkingX = 58;
    public static double parkingY = 56;
    public static int backDelivery = Math.min(PhaseTwoBot.armMax, 1200);

    public PhaseTwoBot bot;
    public OpMode opMode;
    MecanumDrive drive;

    // Constructor - Instantiate this class before waitForStart ==============
    public BackStageBlueCenterPath(OpMode newOpMode, PhaseTwoBot newBot, MecanumDrive newDrive) {
        opMode = newOpMode;
        bot = newBot;
        drive = newDrive;
    }

    // Run this after Start ==================================================
    @Override
    public void runAutoPath() {
        TranslationalVelConstraint slow = new TranslationalVelConstraint(15);
        TranslationalVelConstraint mediumSpeed = new TranslationalVelConstraint(25);


        Actions.runBlocking(drive.actionBuilder(drive.pose)
//                .afterTime(0.0, bot.gripperArm().moveArmToPositionAction(PhaseTwoBot.armDropOne))
//                .splineTo(new Vector2d(pushX + 4.5, pushY), Math.toRadians(pushAngle + 20))
//                .setReversed(true)
//                .splineTo(new Vector2d(initX, initY - 8), Math.toRadians(initAngle - 180))
//                .setReversed(false)
                .splineTo(new Vector2d(pushX, pushY), Math.toRadians(pushAngle), mediumSpeed)
                .stopAndAdd(new SequentialAction(bot.gripperArm().gripperHalfOpenAction(),
                        bot.gripperArm().moveArmToStopAction(1, true)))

                .setReversed(true)
                .afterTime(0.5, bot.gripperArm().moveArmToPositionAction(backDelivery, "start moving", true))
                .splineTo(new Vector2d(26, 42), Math.toRadians(0), mediumSpeed)
                .splineTo(new Vector2d(preDeliveryX, deliveryY), Math.toRadians(0), mediumSpeed)
                .splineTo(new Vector2d(deliveryX, deliveryY), Math.toRadians(0), mediumSpeed)
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

    public void waitForTime(double time) {
        double startTime = opMode.getRuntime() ;
        while (IsTimeUp(startTime,time)){
        }
    }
    public boolean IsTimeUp(double startTime, double runTime) { return opMode.getRuntime()<startTime+runTime ; } // From Owen


}
