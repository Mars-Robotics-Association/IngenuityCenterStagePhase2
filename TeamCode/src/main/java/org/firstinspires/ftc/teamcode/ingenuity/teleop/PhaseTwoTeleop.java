/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.ingenuity.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PhaseTwoBot;

import java.util.ArrayDeque;
import java.util.Queue;

@Config
//@Disabled
public abstract class PhaseTwoTeleop extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    private final boolean useSecondController;
    private final boolean drivingEnabled;


    private GamepadEx driverOp;
    private GamepadEx payloadOp;
    private ToggleButtonReader toggleX;
    private ToggleButtonReader toggleY;
    private ToggleButtonReader toggleStick;
    private ToggleButtonReader toggleB;
    private PhaseTwoBot bot;


    public String lastBumper = "none";

    protected PhaseTwoTeleop(boolean useSecondController, boolean drivingEnabled) {
        this.useSecondController = useSecondController;
        this.drivingEnabled = drivingEnabled;
    }


    /* ===========================================================
     * Code to run ONCE when the driver hits INIT
     ---------------------------------------------------------- */
    @Override
    public void init() {
        // Initialize the gamepad
        driverOp = new GamepadEx(gamepad1);
        payloadOp = useSecondController ? new GamepadEx(gamepad2) : driverOp;
        toggleX = new ToggleButtonReader(payloadOp, GamepadKeys.Button.X);
        toggleY = new ToggleButtonReader(payloadOp, GamepadKeys.Button.Y);
        toggleB = new ToggleButtonReader(payloadOp, GamepadKeys.Button.B);
        toggleStick = new ToggleButtonReader(driverOp, GamepadKeys.Button.LEFT_STICK_BUTTON);

        bot = new PhaseTwoBot(hardwareMap, telemetry, runtime);
        bot.gripperArm();
        bot.ftcLibMecanumDrive();
        bot.droneLauncher();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /* ===========================================================
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     ---------------------------------------------------------- */
    @Override
    public void init_loop() {

    }


    /* ===========================================================
     * Code to run ONCE when the driver hits PLAY
     ---------------------------------------------------------- */
    @Override
    public void start() {
        runtime.reset();
    }

    Queue<Double> timeQueue = new ArrayDeque<>();
    Queue<Double> positionQueue = new ArrayDeque<>();
    double maxRate = 0;

    /* ===========================================================
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     ---------------------------------------------------------- */
    @Override
    public void loop() {
        driverOp.readButtons();

        if (drivingEnabled) {
            bot.ftcLibMecanumDrive().loop(driverOp.isDown(GamepadKeys.Button.A),
                    toggleStick.getState(),
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    driverOp.getRightX());
        }

        toggleStick.readValue();

        if (toggleX.stateJustChanged()) {
            if (toggleX.getState()) {
                bot.gripperArm().closeGripper();
            } else {
                bot.gripperArm().openGripper();
            }
        }
        toggleX.readValue();

        if (payloadOp.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            if (toggleY.getState()) {
                bot.droneLauncher().launchDrone();
            } else {
                bot.droneLauncher().holdDrone();
            }
            toggleY.readValue();
            bot.droneLauncher().loop();

            bot.gripperArm().setPullUpMode(toggleB.getState());
            toggleB.readValue();
        }

        if (bot.isArmEncoderReset() && payloadOp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            bot.gripperArm().lowerArmOneStop();
            lastBumper = "left";
        } else if (bot.isArmEncoderReset() && payloadOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            bot.gripperArm().raiseArmOneStop();
            lastBumper = "right";
        } else if (!payloadOp.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            double netTrigger = payloadOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                    - payloadOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            bot.gripperArm().moveArmManually(netTrigger);
            telemetry.addData("Max arm rate", maxRate);
        }

        double currentTime = runtime.seconds();
        double armPos = bot.gripperArm().getArmPosition();
        timeQueue.offer(currentTime);
        positionQueue.offer(armPos);
        if (timeQueue.size() == 40) {
            double oldTime = timeQueue.poll();
            double oldPosition = positionQueue.poll();
            double rate = Math.abs((armPos - oldPosition) / (currentTime - oldTime));
            if (rate > maxRate) {
                maxRate = rate;
            }
        }

        bot.gripperArm().loop();

//        if (driverOp.isDown(GamepadKeys.Button.DPAD_UP)) {
//            bot.winch().windUp();
//        } else if (driverOp.isDown(GamepadKeys.Button.DPAD_DOWN)) {
//            bot.winch().windDown();
//        } else {
//            bot.winch().stop();
//        }
//        bot.winch().writeTelemetry();

        if (driverOp.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            if (driverOp.wasJustReleased(GamepadKeys.Button.A)) {
                bot.ftcLibMecanumDrive().resetHeading();
            }
        }

        if (payloadOp.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
            double netTrigger = payloadOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                    - payloadOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            bot.winch().setPower(-netTrigger);
        }
    }

    /* ===========================================================
     * Code to run ONCE after the driver hits STOP
     ---------------------------------------------------------- */
    @Override
    public void stop() {
    }

}
