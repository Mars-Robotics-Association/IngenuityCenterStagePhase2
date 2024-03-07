package org.firstinspires.ftc.teamcode.ingenuity.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.Alliance;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.StagePosition;
import org.firstinspires.ftc.teamcode.ingenuity.autoPaths.TheOnePathToRuleThemAll;

@Config
@Autonomous(name = "BackStageRedAuto", group = "Auto 3.0 development")
public final class AutoBackStageRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TheOnePathToRuleThemAll one = new TheOnePathToRuleThemAll(Alliance.RED, StagePosition.BACK, hardwareMap, telemetry, new ElapsedTime());

        waitForStart(); // ========================================================================

        one.start(this::sleep, this::updateTelemetry, this::opModeIsActive);
    }
}
