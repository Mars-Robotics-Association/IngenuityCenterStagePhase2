package org.firstinspires.ftc.teamcode.ingenuity.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PhaseTwoBot;
import org.firstinspires.ftc.teamcode.PropDetection;
import org.firstinspires.ftc.teamcode.PropPosition;
import org.firstinspires.ftc.teamcode.TimeoutAction;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.Alliance;


@Config
@Autonomous(name = "AutoPOC2", group = "Auto 3.0 development")
public final class AutoPOC2 extends LinearOpMode {
    public static double initX = 12;
    public static double initY = 61;
    public static double initAngle = 270;
    public static double deliveryX = 48;
    public static double preDeliveryX = deliveryX - 6.5;
    public static double parkingX = 58;
    public static double parkingY = 56;
    public static Alliance alliance = Alliance.RED;

    public PhaseTwoBot bot;
    private MecanumDrive drive;
    private PropPosition propPosition;
    private PropDetection propDetector;

    private TranslationalVelConstraint slow = new TranslationalVelConstraint(15);
    private int backDelivery = Math.min(PhaseTwoBot.armMax, 2110);

    private Vector2d relCoords(double xOffset, double yOffset) {
        if (alliance == Alliance.BLUE) {
            return new Vector2d(initX + xOffset, initY + yOffset);
        } else {
            return new Vector2d(initX + xOffset, 0.0 - initY - yOffset);
        }
    }

    private Vector2d absCoords(double x, double y) {
        return new Vector2d(x, alliance == Alliance.BLUE ? y : 0.0 - y);
    }

    private double absHeading(double degrees) {
        return Math.toRadians(alliance == Alliance.BLUE ? degrees : mirrorAngle(degrees));
    }

    private double relHeading(double degreesOffset) {
        return Math.toRadians((alliance == Alliance.BLUE ? initAngle : mirrorAngle(initAngle)) + (alliance == Alliance.BLUE ? degreesOffset : 0.0 - degreesOffset));
    }

    public static double reverseAngle(double degrees) {
        return (degrees + 180.0) % 360.0;
    }

    public static double mirrorAngle(double degrees) {
        return ((360.0) - degrees) % 360.0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        propDetector = new PropDetection(hardwareMap, telemetry);
        bot = new PhaseTwoBot(hardwareMap, telemetry, new ElapsedTime());

        Actions.runBlocking(new SequentialAction(
                bot.AutonomousInitActions()
        ));

        drive = new MecanumDrive(hardwareMap, new Pose2d(absCoords(initX, initY), absHeading(initAngle)));

        waitForStart(); // ========================================================================

        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .splineTo(relCoords(0, -8), relHeading(0))  // Drive closer to the team prop to get a better view
                .afterTime(0, bot.gripperArm().setWristFlatZero())      // Put the gripper wrist in ground position
                .build());
        sleep(1000);  // Wait for the vision processor to scan
        propPosition = propDetector.propTfod();
        updateTelemetry(telemetry);

        Actions.runBlocking(bot.gripperArm().moveArmToPositionAction(PhaseTwoBot.armDropOne));
        TrajectoryActionBuilder trajBuilder = drive.actionBuilder(drive.pose);
        trajBuilder = placePurplePixel(trajBuilder);
        trajBuilder = placeYellowPixel(trajBuilder);
        trajBuilder = park(trajBuilder);
        Actions.runBlocking(trajBuilder.build());
    }

    private TrajectoryActionBuilder placePurplePixel(TrajectoryActionBuilder trajBuilder) {
        switch (propPosition) {
            case MIDDLE:
                trajBuilder = trajBuilder.splineTo(relCoords(+2, -29), relHeading(20));
                break;
            case RIGHT:
                trajBuilder = trajBuilder.splineTo(relCoords(-5.5, -23), relHeading(-40));
                break;
            default:
                trajBuilder = trajBuilder.splineTo(relCoords(10, -23), relHeading(30));
                break;
        }
        trajBuilder = trajBuilder
                .afterTime(0, new SequentialAction(bot.gripperArm().gripperHalfOpenAction()))
                .afterTime(0.5, new SequentialAction(bot.gripperArm().gripperCloseAction()))
                .setReversed(true)
                .splineTo(relCoords(0, -8), absHeading(reverseAngle(initAngle)));
        return trajBuilder;
    }

    private TrajectoryActionBuilder placeYellowPixel(TrajectoryActionBuilder trajBuilder) {
        double deliveryY = propPosition == PropPosition.MIDDLE ? 35 :
                propPosition == PropPosition.RIGHT ? 27 : 41;

        return trajBuilder
                .splineTo(absCoords(26, 49), absHeading(0))
                .splineTo(absCoords(preDeliveryX, deliveryY), absHeading(0))
                .afterTime(0.15, bot.gripperArm().moveArmToPositionAction(backDelivery, "start moving", true))
                .splineTo(absCoords(deliveryX, deliveryY), absHeading(0), slow)
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
                .splineTo(absCoords(preDeliveryX, deliveryY), absHeading(180));
    }

    private TrajectoryActionBuilder park(TrajectoryActionBuilder trajBuilder) {
        return trajBuilder
                .strafeTo(absCoords(preDeliveryX, parkingY))
                .setReversed(true)
                .splineTo(absCoords(parkingX, parkingY), absHeading(0));
    }
}
