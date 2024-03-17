package org.firstinspires.ftc.teamcode.ingenuity.testing;

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
import org.firstinspires.ftc.teamcode.PropPosition;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.Alliance;

@Config
@Autonomous(name = "Prop Detect RED", group = "Auto 3.0 development")
public final class PropDetectRed extends LinearOpMode {
    public static double initX = -36;
    public static double initY = 61;
    public static double initAngle = -90;
    public static double pushX = initX;
    public static double pushY = initY - 35.5;
    public static double pushAngle = initAngle;
    public static double invPushAngle = ((int) pushAngle + 180) % 360;
    public static double deliveryX = 48;
    public static double preDeliveryX = deliveryX - 6.5;
    public static double deliveryY = 35;
    public static double parkingX = 58;
    public static double parkingY = 56;
    public static int backDelivery = Math.min(PhaseTwoBot.armMax, 2110);

    PropPosition propPosition ;
    PropDetection propDetector ;

    @Override
    public void runOpMode() throws InterruptedException {
        propDetector = new PropDetection(hardwareMap, telemetry);

        PhaseTwoBot bot = new PhaseTwoBot(hardwareMap, telemetry, new ElapsedTime());
        TranslationalVelConstraint slow = new TranslationalVelConstraint(15);

        telemetry.addLine("Check 1") ;
        updateTelemetry(telemetry);


        Actions.runBlocking(new SequentialAction(
                bot.AutonomousInitActions(),
                bot.gripperArm().moveArmToPositionAction(PhaseTwoBot.armDropOne)
        ));

        telemetry.addLine("Check 2") ;

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(initX, initY, Math.toRadians(initAngle)));

        updateTelemetry(telemetry);

        waitForStart();  // ======================================================================================


        telemetry.addLine("Check 3") ;
        updateTelemetry(telemetry);

        Actions.runBlocking(drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(initX, 55), Math.toRadians(-90))
                .build());


        sleep(5000);

        telemetry.addLine("Check 4") ;

        propPosition = propDetector.propTfod(Alliance.RED);
        updateTelemetry(telemetry);

//            PoseStorage.currentPose = drive.pose;
        while (opModeIsActive()) {}
    }
}
