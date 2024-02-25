package org.firstinspires.ftc.teamcode.ingenuity.autoOLD;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PhaseTwoBot;
import org.firstinspires.ftc.teamcode.TimeoutAction;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config
@Autonomous(name = "left 3 front stage blue", group = "Auto 3.0 development")
@Disabled
public final class FrontStageBlueLeft3 extends LinearOpMode {
    public static double initPause = 5.0;
    public static double initX = -36;
    public static double initY = 61;
    public static double initAngle = 270;
    public static double pushX = initX + 8.5;
    public static double pushY = initY - 26;
    public static double pushAngle = initAngle + 40;
    public static double invPushAngle = ((int) pushAngle + 180 + 10) % 360;
    public static double backOffXby = 4.5;
    public static double backOffYby = 4.5;
    public static double centerLaneY = 10;
    public static double deliveryX = 48;
    public static double preDeliveryX = deliveryX - 6.5;
    public static double deliveryY = 35;
    public static double parkingX = 58;
    public static double parkingY = 8;
    public static int backDelivery = Math.min(PhaseTwoBot.armMax, 1200);
    public static double armDelay = 4.25;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            PhaseTwoBot bot = new PhaseTwoBot(hardwareMap, telemetry, new ElapsedTime());
            TranslationalVelConstraint slow = new TranslationalVelConstraint(15);

            Actions.runBlocking(new SequentialAction(
                    bot.AutonomousInitActions(),
                    bot.gripperArm().moveArmToPositionAction(PhaseTwoBot.armDropOne)
            ));

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(initX, initY, Math.toRadians(initAngle)));

            waitForStart();

            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .stopAndAdd(new SleepAction(initPause))
                    .splineTo(new Vector2d(initX, initY - 8), Math.toRadians(initAngle))
                    .afterTime(0.0, bot.gripperArm().moveArmToStopAction(1, true))
                    .splineTo(new Vector2d(pushX, pushY), Math.toRadians(pushAngle))
                    .afterTime(0.85, bot.gripperArm().moveArmToPositionAction(PhaseTwoBot.armDropOne))
                    .setReversed(true)
                    .afterTime(armDelay, bot.gripperArm().moveArmToPositionAction(backDelivery, "start moving", true))
                    .splineTo(new Vector2d(pushX - backOffXby, pushY + backOffYby), Math.toRadians(invPushAngle))
                    .splineTo(new Vector2d(-48, 46), Math.toRadians(180))
                    .splineTo(new Vector2d(-55, pushY - 5), Math.toRadians(initAngle))
                    .splineTo(new Vector2d(-36, centerLaneY), Math.toRadians(0))
                    .splineTo(new Vector2d(25, centerLaneY), Math.toRadians(0))
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

//            PoseStorage.currentPose = drive.pose;
        } else {
            throw new AssertionError();
        }
    }
}
