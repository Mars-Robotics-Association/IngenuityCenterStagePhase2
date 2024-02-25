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
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.BackStageBlueCenterPath;

@Config
@Autonomous(name = "BackStageBlueAuto", group = "Auto 3.0 development")
public final class AutoBackStageBlue extends LinearOpMode {
    BackStageBlueCenterPath autonomousPath ;
    public static double initX = 12;
    public static double initY = 61;
    public static double initAngle = 270;

    public PhaseTwoBot bot ;
    MecanumDrive drive ;
    int propPosition ;
    PropDetection propDetector ;


    @Override
    public void runOpMode() throws InterruptedException {
        propDetector = new PropDetection(this);
        bot = new PhaseTwoBot(hardwareMap, telemetry, new ElapsedTime());
        TranslationalVelConstraint slow = new TranslationalVelConstraint(15);
        Actions.runBlocking(new SequentialAction(
                bot.AutonomousInitActions()
        ));
        drive = new MecanumDrive(hardwareMap, new Pose2d(initX, initY, Math.toRadians(initAngle)));

        waitForStart(); // ========================================================================

        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(initX, 55), Math.toRadians(-90))
                        .afterTime(0,bot.gripperArm().setWristFlatZero())
                .build());
        sleep(5000);  // TODO: Lower this for Backstage
        propPosition = propDetector.propTfod();
        updateTelemetry(telemetry);

        switch (propPosition) {
            case 0: // Middle
                autonomousPath = new BackStageBlueCenterPath(this, bot, drive) ;
                break ;
            case 1: // Right

                break ;
            default: // Left

                break ;
        }

        autonomousPath = new BackStageBlueCenterPath(this, bot, drive) ;

        if (opModeIsActive()) autonomousPath.runAutoPath();
    }
}
