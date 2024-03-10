package org.firstinspires.ftc.teamcode.ingenuity.autoPaths;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PhaseTwoBot;
import org.firstinspires.ftc.teamcode.PropDetection;
import org.firstinspires.ftc.teamcode.PropPosition;
import org.firstinspires.ftc.teamcode.TimeoutAction;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Supplier;

@Config
public class TheOnePathToRuleThemAll {
    private final Alliance alliance;
    private final StagePosition stagePosition;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final PhaseTwoBot bot;
    private final MecanumDrive drive;


    public static final double directionAudience = 180;
    public static final double directionBackdrop = 0;
    public static final double directionOpponent = 270;
    public static final double directionSelf = 90;

    public static double slowVel = 15;
    public static double mediumVel = 25;

    private final VelConstraint slow;
    private final VelConstraint medium;

    private final double initX;
    private final double initY;
    private final double initAngle;
    private final double parkingY;

    public static double initXFront = -36;
    public static double initXBack = 12;
    public static double centerLaneY = 10;
    public static double relTurn = -23;
    public static double deliveryX = 46;
    public static double preDeliveryX = deliveryX - 4;
    public static double parkingX = 58;
    public static double parkingYFront = 8;
    public static double parkingYBack = 56;
    public static int backDelivery = Math.min(PhaseTwoBot.armMax, 2350);
    private PropDetection propDetector;
    private PropPosition propPosition;


    public TheOnePathToRuleThemAll(Alliance alliance, StagePosition stagePosition, HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime elapsedTime) {
        this.alliance = alliance;
        this.stagePosition = stagePosition;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initX = stagePosition == StagePosition.BACK ? initXBack : initXFront;
        initY = 61;
        initAngle = 270;
        parkingY = stagePosition == StagePosition.BACK ? parkingYBack : parkingYFront;

        propDetector = new PropDetection(hardwareMap, telemetry);
        bot = new PhaseTwoBot(hardwareMap, telemetry, elapsedTime);
        drive = new MecanumDrive(hardwareMap, getStartingPose());

        Actions.runBlocking(new SequentialAction(
                bot.AutonomousInitActions()
        ));

        slow = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(slowVel),
                new AngularVelConstraint(Math.PI / 2.0)
        ));
        medium = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(mediumVel),
                new AngularVelConstraint(Math.PI * 0.55)
        ));
    }

    public static double reverseAngle(double degrees) {
        return (degrees + 180.0) % 360.0;
    }

    public static double mirrorAngle(double degrees) {
        return ((360.0) - degrees) % 360.0;
    }

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

    public void start(Consumer<Long> sleep, Consumer<Telemetry> updateTelemetry, Supplier<Boolean> opModeIsActive) {
        Actions.runBlocking(new TimeoutAction(bot.gripperArm().lowerArmToLimit(), 1.0));
        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .afterTime(0, bot.gripperArm().gripperCloseAction())
                .splineTo(relCoords(0, -12), relHeading(0), medium)  // Drive closer to the team prop to get a better view
                .afterTime(0, bot.gripperArm().setWristFlatZero())      // Put the gripper wrist in ground position
                .build());
        sleep.accept(1000L);
        if (opModeIsActive.get()) {
            propPosition = propDetector.propTfod(alliance);
            updateTelemetry.accept(telemetry);
        }
        if (opModeIsActive.get()) {
            Actions.runBlocking(bot.gripperArm().moveArmToPositionAction(PhaseTwoBot.armDropOne));
            Actions.runBlocking(afterScan(drive.actionBuilder(drive.pose)).build());
        }
    }

    private TrajectoryActionBuilder afterScan(TrajectoryActionBuilder trajBuilder) {
        // reverse left and right if we're on the red alliance
        propPosition = alliance == Alliance.BLUE ? propPosition :
                propPosition == PropPosition.LEFT ? PropPosition.RIGHT :
                        propPosition == PropPosition.RIGHT ? PropPosition.LEFT :
                                PropPosition.MIDDLE;

        trajBuilder = placePurplePixel(trajBuilder);
        if (stagePosition == StagePosition.FRONT) {
            trajBuilder = driveFromFrontToBack(trajBuilder);
        } else {
            trajBuilder = driveFromBackToBack(trajBuilder);
        }
        trajBuilder = placeYellowPixel(trajBuilder);
        trajBuilder = park(trajBuilder);
        return trajBuilder;
    }

    public Pose2d getStartingPose() {
        return new Pose2d(relCoords(0, 0), relHeading(0));
    }

    private TrajectoryActionBuilder driveFromFrontToBack(TrajectoryActionBuilder trajBuilder) {
        trajBuilder = trajBuilder
                .setReversed(true)
                .splineTo(relCoords(0, -12), absHeading(reverseAngle(initAngle)))
                .setReversed(false);

        if (propPosition == PropPosition.MIDDLE) {
            trajBuilder = trajBuilder
                    .splineTo(absCoords(initX - 12, 24), relHeading(-1), medium)
                    .splineTo(absCoords(initX - 20, centerLaneY), absHeading(directionAudience), medium);
        } else {
            trajBuilder = trajBuilder
                    .splineTo(absCoords(initX, 24), absHeading(directionOpponent), medium)
                    .splineTo(absCoords(initX - 12, centerLaneY), absHeading(directionAudience), medium);
        }
        trajBuilder = trajBuilder
                .setReversed(true)
                .splineTo(absCoords(initX, centerLaneY), absHeading(directionBackdrop), medium)
                .splineTo(absCoords(22, centerLaneY), absHeading(directionBackdrop));
        return trajBuilder;
    }

    private TrajectoryActionBuilder driveFromBackToBack(TrajectoryActionBuilder trajBuilder) {
        return trajBuilder
                .setReversed(true)
                .splineTo(
                        propPosition == PropPosition.RIGHT ? absCoords(21, 42) :
                                propPosition == PropPosition.MIDDLE ? absCoords(21, 42) :
                                        absCoords(18, 55), absHeading(directionBackdrop));
    }

    private TrajectoryActionBuilder placePurplePixel(TrajectoryActionBuilder trajBuilder) {
        switch (propPosition) {
            case MIDDLE:
                trajBuilder = trajBuilder
                        .splineTo(relCoords(+1.5, -29), relHeading(15), medium);
                break;
            case RIGHT:
                trajBuilder = trajBuilder
                        .splineTo(relCoords(-4.5, -22), relHeading(-40), medium);
                break;
            default:
                trajBuilder = trajBuilder
                        .splineTo(relCoords(6, -23), relHeading(30), medium);
                break;
        }
        trajBuilder = trajBuilder
                .stopAndAdd(new SequentialAction(
                        bot.gripperArm().gripperHalfOpenAction(),
                        bot.gripperArm().gripperCloseAction(),
                        bot.gripperArm().setWristTuckedUp()
                ));
        return trajBuilder;
    }

    private TrajectoryActionBuilder placeYellowPixel(TrajectoryActionBuilder trajBuilder) {
        double deliveryY = propPosition == PropPosition.MIDDLE ? 35 :
                propPosition == PropPosition.RIGHT ? 27 : 41;

        return trajBuilder

                .splineTo(absCoords(preDeliveryX, deliveryY), absHeading(directionBackdrop), medium) // line up
                .stopAndAdd(new NullAction())
                .splineTo(absCoords(deliveryX, deliveryY), absHeading(directionBackdrop), slow) // final approach
                .afterTime(0, new TimeoutAction(bot.gripperArm().moveArmToPositionAction(backDelivery, "start moving", true), 2.0))


                .stopAndAdd(new SequentialAction(
                        new TimeoutAction(bot.gripperArm().moveArmToPositionAction(backDelivery, "finish moving", true), 1.5),
                        new SleepAction(0.15),
                        bot.gripperArm().gripperOpenAction(),
                        new SleepAction(0.25),
                        bot.gripperArm().setWristTuckedUp(),
                        bot.gripperArm().moveArmToStopAction(1, false)))
                .setReversed(false)
                .afterTime(0.0, new SequentialAction(
                        bot.gripperArm().moveArmToStopAction(0),
                        bot.gripperArm().lowerArmToLimit()
                ))
                .splineTo(absCoords(preDeliveryX, deliveryY), absHeading(directionAudience), medium);
    }

    private TrajectoryActionBuilder park(TrajectoryActionBuilder trajBuilder) {
        return trajBuilder
                .strafeTo(absCoords(preDeliveryX, parkingY))
                .setReversed(true)
                .splineTo(absCoords(parkingX, parkingY), absHeading(directionBackdrop), medium);
    }

}
