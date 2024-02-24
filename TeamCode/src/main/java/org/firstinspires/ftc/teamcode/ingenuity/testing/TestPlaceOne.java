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

@Autonomous(name = "test place one", group = "testing opmodes")
@Config
public class TestPlaceOne extends LinearOpMode {

    private PhaseTwoBot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            bot = new PhaseTwoBot(hardwareMap, telemetry, new ElapsedTime());

            Actions.runBlocking(bot.AutonomousInitActions());

            waitForStart();

            Actions.runBlocking(new SequentialAction(
                    bot.gripperArm().moveArmToPositionAction(PhaseTwoBot.armDropOne, "step 1"),
//                    bot.gripperArm().applyAutoWristAction(),
                    new SleepAction(0.5),
                    bot.gripperArm().gripperOpenAction(),
                    new SleepAction(0.3),
                    bot.gripperArm().gripperCloseAction(),
                    bot.gripperArm().setWristTuckedUp(),
                    bot.gripperArm().moveArmToStopAction(1, "step 2"),
//                    bot.gripperArm().applyAutoWristAction(),
                    bot.gripperArm().gripperOpenAction(),
                    new SleepAction(.5),
                    //new ParallelAction(
                            bot.gripperArm().gripperCloseAction(),
                            bot.gripperArm().moveArmToStopAction(0, "step 3"),
                    //),
                    bot.gripperArm().lowerArmToLimit()
            ));
        } else {
            throw new AssertionError();
        }
    }
}
