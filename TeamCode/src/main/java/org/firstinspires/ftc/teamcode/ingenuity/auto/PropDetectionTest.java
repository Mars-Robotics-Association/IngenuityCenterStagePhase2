package org.firstinspires.ftc.teamcode.ingenuity.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PhaseTwoBot;
import org.firstinspires.ftc.teamcode.PropDetection;
import org.firstinspires.ftc.teamcode.TimeoutAction;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config
@Autonomous(name = "Prop Detection Test", group = "Auto 3.0 development")
public final class PropDetectionTest extends LinearOpMode {
    public static double initX = 12;
    public static double initY = 61;
    public static double initAngle = 270;
    public static double pushX = initX;
    public static double pushY = initY - 35.5;
    public static double pushAngle = initAngle;
    public static double invPushAngle = ((int) pushAngle + 180) % 360;
    public static double deliveryX = 48;
    public static double preDeliveryX = deliveryX - 6.5;
    public static double deliveryY = 35;
    public static double parkingX = 58;
    public static double parkingY = 56;
    public static int backDelivery = 2250;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            PropDetection propDetector = new PropDetection(this);


            PhaseTwoBot bot = new PhaseTwoBot(hardwareMap, telemetry);
            TranslationalVelConstraint slow = new TranslationalVelConstraint(15);

            Actions.runBlocking(new SequentialAction(
                    bot.AutonomousInitActions(),
                    bot.gripperArm().moveArmToPositionAction(PhaseTwoBot.armDropOne)
            ));

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(initX, initY, Math.toRadians(initAngle)));


            sleep(5000);
            int propPosition = propDetector.propTfod();
            updateTelemetry(telemetry);
            waitForStart();

            Actions.runBlocking(drive.actionBuilder(drive.pose)
                    .afterTime(0.0, bot.gripperArm().moveArmToStopAction(1, true))
                    .splineTo(new Vector2d(pushX, pushY), Math.toRadians(pushAngle))
                    .setReversed(true)
                    .afterTime(0.0, bot.gripperArm().moveArmToPositionAction(backDelivery, "start moving", true))
                    .splineTo(new Vector2d(11, 33), Math.toRadians(invPushAngle))
                    .splineTo(new Vector2d(26, 42), Math.toRadians(0))
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
