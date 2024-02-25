package org.firstinspires.ftc.teamcode.ingenuity.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PhaseTwoBot;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name = "move wrist to ground", group = "testing opmodes")
@Config
public class MoveWristToGround extends LinearOpMode {


    private PhaseTwoBot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            bot = new PhaseTwoBot(hardwareMap, telemetry, new ElapsedTime());


            waitForStart();

            Actions.runBlocking(new SequentialAction(
                    bot.gripperArm().setWristFlatZero(),
                    new SleepAction(120)
            ));
        } else {
            throw new AssertionError();
        }
    }
}
