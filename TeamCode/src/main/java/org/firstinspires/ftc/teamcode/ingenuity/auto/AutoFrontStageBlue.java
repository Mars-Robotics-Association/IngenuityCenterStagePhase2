package org.firstinspires.ftc.teamcode.ingenuity.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PhaseTwoBot;
import org.firstinspires.ftc.teamcode.PropDetection;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.AutoPath;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.BackStageBlueCenterPath;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.BackStageBlueLeftPath;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.BackStageBlueRightPath;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.FrontStageBlueCenterPath;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.FrontStageBlueLeftPath;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.FrontStageBlueRightPath;

@Config
@Autonomous(name = "FrontStageBlueAuto", group = "Auto 3.0 development")
public final class AutoFrontStageBlue extends LinearOpMode {
    AutoPath autonomousPath;
    public static double initX = -36;
    public static double initY = 61;
    public static double initAngle = 270;

    public PhaseTwoBot bot;
    MecanumDrive drive;
    int propPosition;
    PropDetection propDetector;


    @Override
    public void runOpMode() throws InterruptedException {
        propDetector = new PropDetection(this);
        bot = new PhaseTwoBot(hardwareMap, telemetry, new ElapsedTime());
        TranslationalVelConstraint slow = new TranslationalVelConstraint(15);
        Actions.runBlocking(new SequentialAction(
                bot.AutonomousInitActions()
        ));
        bot.gripperArm().closeGripper();
        drive = new MecanumDrive(hardwareMap, new Pose2d(initX, initY, Math.toRadians(initAngle)));

        waitForStart(); // ========================================================================

        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(initX, 53), Math.toRadians(-90))
                .afterTime(0, bot.gripperArm().setWristFlatZero())
                .build());
        sleep(5000);  // TODO: Lower this for Backstage
        propPosition = propDetector.propTfod();
        updateTelemetry(telemetry);

        Actions.runBlocking(bot.gripperArm().moveArmToPositionAction(PhaseTwoBot.armDropOne));
        switch (propPosition) {
            case 0: // Middle
                autonomousPath = new FrontStageBlueCenterPath(this, bot, drive);
                break;
            case 1: // Right
                autonomousPath = new FrontStageBlueRightPath(this, bot, drive);
                break;
            default: // Left
                autonomousPath = new FrontStageBlueLeftPath(this, bot, drive);
                break;
        }

        if (opModeIsActive()) autonomousPath.runAutoPath();
    }
}
