package org.firstinspires.ftc.teamcode.ingenuity.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.AutoPath;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.BackStageBlueCenterPath;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.BackStageBlueLeftPath;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.BackStageBlueRightPath;

@Config
@Autonomous(name = "BackStageBlueAuto", group = "Auto 3.0 development")
public final class AutoBackStageBlue extends LinearOpMode {
    AutoPath autonomousPath;
    public static double initX = 12;
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
                .splineTo(new Vector2d(initX, 53), Math.toRadians(-90))  // Drive closer to the team prop to get a better view
                .afterTime(0, bot.gripperArm().setWristFlatZero())      // Put the gripper wrist in ground position
                .build());
        sleep(1000);  // Wait for the vision processor to scan TODO: Lower this for Backstage
        propPosition = propDetector.propTfod();   // Get the prop position: -1 = Left, 0 = Middle, 1 = Right
        updateTelemetry(telemetry);

        Actions.runBlocking(bot.gripperArm().moveArmToPositionAction(PhaseTwoBot.armDropOne));
        switch (propPosition) {
            case 0: // Middle
                autonomousPath = new BackStageBlueCenterPath(this, bot, drive);
                break;
            case 1: // Right
                autonomousPath = new BackStageBlueRightPath(this, bot, drive);
                break;
            default: // Left
                autonomousPath = new BackStageBlueLeftPath(this, bot, drive);
                break;
        }

        if (opModeIsActive()) autonomousPath.runAutoPath();
    }
}
