package org.firstinspires.ftc.teamcode.ingenuity.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PhaseTwoBot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name = "test arm move", group = "testing opmodes")
@Config
public class TestArmMove extends LinearOpMode {
    public static int moveTo = 2500;
    public static int moveTo2 = 400;
    public static double sleepTime = 1.0;

    private PhaseTwoBot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            bot = new PhaseTwoBot(hardwareMap, telemetry, new ElapsedTime());

            Actions.runBlocking(bot.AutonomousInitActions());

            waitForStart();

            Actions.runBlocking(new SequentialAction(
//                    bot.gripperArm().moveArmToStopAction(2, true),
//                    new SleepAction(0.2),
//                    bot.gripperArm().gripperOpenAction(),
//                    new SleepAction(0.5),
//                    bot.gripperArm().setWristTuckedUp(),
//                    new ParallelAction(
//                            bot.gripperArm().moveArmToStopAction(0, false),
//                            new SequentialAction(
//                                    new SleepAction(.6),
//                                    bot.gripperArm().gripperCloseAction()
//                            )
//                    )
                    new SleepAction(sleepTime),
                    bot.gripperArm().moveArmToPositionAction(moveTo, "step 1"),
                    new SleepAction(sleepTime),
                    bot.gripperArm().moveArmToPositionAction(moveTo2, "step 2"),
                    new SleepAction(sleepTime),
                    bot.gripperArm().moveArmToPositionAction(moveTo, "step 3"),
                    new SleepAction(sleepTime),
                    bot.gripperArm().moveArmToPositionAction(0, "step 4"),
                    bot.gripperArm().lowerArmToLimit()
            ));
        } else {
            throw new AssertionError();
        }
    }
}
