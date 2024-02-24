package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.security.InvalidParameterException;
import java.util.concurrent.TimeUnit;

@Config
public class PhaseTwoBot {
    public static int closeToZero = 550;
    public static double noWakeSpeed = -0.35;
    public static boolean autoWrist = true;
    public static int gripperCloseTime = 350;
    public static int gripperOpenTime = 300;
    public static double wristFlatZero = 0.51;
    public static double wristTuckedUp = 0.85;
    public static double gripperOpenPosition = 0.55;
    public static double gripperHalfOpenPosition = .515;
    public static double gripperClosedPosition = .47;
    public static double wristPosition = 0.5;
    public static double gripperPosition = gripperOpenPosition;
    public static double armExpo = 0.01;
    public static double wristPullUp = 0.35;

    public static int armRearPickupStart = 2800;
    public static double wristMaxInversion = 0.93;
    public static double wristRearPickUpStart = Math.min(0.93, wristMaxInversion);
    public static double wristRearPickUp = 0.83;
    public static int armMax = 2900;

    public static int armDropOne = 50;


    public static double positionCoefficient = 0.00075;
    public static double positionIntegralCoeff = 0.18;
    public static double positionTolerance = 3;
    public static double rampSlope = 1900;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final ElapsedTime runtime;

    private boolean armEncoderWasReset = false;

    public PhaseTwoBot(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime runtime) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.runtime = runtime;
    }

    public Action AutonomousInitActions() {
        gripperArm();

        return new SequentialAction(
                gripperArm().gripperCloseAction(),
                new SleepAction(0.3),
                gripperArm().setWristTuckedUp(),
                droneLauncher().stowLauncherArmAction(),
                gripperArm().lowerArmToLimit()
        );
    }

    private GripperArm gripperArm;

    public GripperArm gripperArm() {
        if (gripperArm == null) {
            gripperArm = new GripperArm();
            gripperArm.init();
        }
        return gripperArm;
    }


    public class GripperArm {
        private WormGroup armMotor;
        //        private Servo gripper;
//        private Servo wrist;
        private Timing.Timer gripperCloseTimer;
        private Timing.Timer gripperOpenTimer;
        private TouchSensor touchSensor;  // Touch sensor Object
        private Motor.RunMode armRunMode = Motor.RunMode.RawPower;

        private int motorFollowerErrorSum = 0;

        private boolean pullUpMode = false;

        private int armSetpointIdx = -1;
        //private int[] armStops = {0, 289, 1150, 1700, 6000};
        private int armApex = 1417;


        private int armMaxFlat = 168;
        private double wristMaxFlat = 0.47;
        public int armMinScore = 239;
        private double wristMinScore = 0.65;
        private int armMaxFrontScore = 870;
        private double wristMaxFrontScore = 0.58;
        private int armMaxReach = 1830;
        private double wristMaxReach = 0.4;
        private int armTopBackScore = 1985;
        private double wristTopBackScore = Math.min(0.93, wristMaxInversion);
        private int armBottomBackScore = 2665;
        private double wristBottomBackScore = 0.8;

        private int[] armStops = {0, (armMinScore + armMaxFrontScore) / 2, (armTopBackScore + armBottomBackScore) / 2};

        private boolean lowerLimit;

        private CancelableAction armMovementAction = null;

        private double linearInterpolation(double x, double minX, double maxX, double minY, double maxY) {
            double slope = (maxY - minY) / (maxX - minX);
            return minY + slope * (x - minX);
        }

        public int getArmPosition() {
            return armMotor.getCurrentPosition();
        }

        private double wristServoValue(int armTicks) {
            if (armTicks < armMaxFlat && gripperPosition == gripperClosedPosition && gripperCloseTimer.done()) {
                return wristTuckedUp;
            } else if (armTicks <= armMaxFlat) {
                return linearInterpolation(armTicks, 0, armMaxFlat, wristFlatZero, wristMaxFlat);
            } else if (armTicks < armMinScore) {
                return wristMinScore;
            } else if (armTicks < armMaxFrontScore) {
                return linearInterpolation(armTicks, armMinScore, armMaxFrontScore, wristMinScore, wristMaxFrontScore);
            } else if (armTicks <= armApex) {
                return pullUpMode ? wristPullUp : wristMaxFrontScore;
            } else if (armTicks < armTopBackScore) {
                return pullUpMode ? wristPullUp : wristTopBackScore;
            } else if (armTicks < (armBottomBackScore + armRearPickupStart) / 2) {
                return Math.min(wristMaxInversion, linearInterpolation(armTicks, armTopBackScore, armBottomBackScore, wristTopBackScore, wristBottomBackScore));
            } else {
                return Math.min(wristMaxInversion, linearInterpolation(armTicks, armRearPickupStart, armMax, wristRearPickUpStart, wristRearPickUp));
            }
        }

        public void init() {
            WormMotor wm0 = new WormMotor(hardwareMap, "rightArmMotor", Motor.GoBILDA.RPM_1150);
            WormMotor wm1 = new WormMotor(hardwareMap, "leftArmMotor", Motor.GoBILDA.RPM_1150);
            armMotor = new WormGroup(telemetry, wm0, wm1);

            wm0.setInverted(false);
            wm1.setInverted(false);

//            gripper = hardwareMap.servo.get("gripper");
//            wrist = hardwareMap.servo.get("wrist");


            armMotor.setPositionPI(positionCoefficient, positionIntegralCoeff);
            armMotor.setPositionTolerance(positionTolerance);

            gripperCloseTimer = new Timing.Timer(gripperCloseTime, TimeUnit.MILLISECONDS);
            gripperOpenTimer = new Timing.Timer(gripperOpenTime, TimeUnit.MILLISECONDS);

            touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");
        }

        private void setArmRunMode(Motor.RunMode runMode) {
            if (runMode != armRunMode) {
                armRunMode = runMode;
                //armMotor.setRunMode(runMode);
            }
        }

        public Motor.RunMode getArmRunMode() {
            return armRunMode;
        }

        public void closeGripper() {
            gripperCloseTimer.start();
            gripperOpenTimer.pause();
            gripperPosition = gripperClosedPosition;
        }

        public void halfOpenGripper() {
            gripperCloseTimer.pause();
            gripperOpenTimer.start();
            gripperPosition = gripperHalfOpenPosition;
        }

        public void openGripper() {
            gripperPosition = gripperOpenPosition;
        }

        public void setPullUpMode(boolean value) {
            pullUpMode = value;
        }

        private int lastStopUp() {
            int curPos = armMotor.getCurrentPosition();
            int i;
            for (i = 0; i < armStops.length; i++) {
                if (curPos < armStops[i]) return i;
            }
            return i;
        }

        private int nextStopDown() {
            int curPos = armMotor.getCurrentPosition();
            int i;
            for (i = armStops.length - 1; i >= 0; i--) {
                if (curPos > armStops[i]) return i;
            }
            return i;
        }

        private void raiseArmToNextStop() {
            if (armSetpointIdx == -1) {
                armSetpointIdx = lastStopUp();
            }
            if (this.armSetpointIdx < armStops.length - 1) {
                armSetpointIdx += 1;
                telemetry.addData("go to stop", armStops[armSetpointIdx]);
                armMotor.setTargetPosition(armStops[armSetpointIdx]);
                armMotor.moveArmToPosInit(armStops[armSetpointIdx], runtime.seconds());
            }
        }

        private void lowerArmToNextStop() {
            if (armSetpointIdx == -1) {
                armSetpointIdx = nextStopDown() + 1;
            }
            if (armSetpointIdx > 0) {
                armSetpointIdx -= 1;
                armMotor.setTargetPosition(armStops[armSetpointIdx]);
                armMotor.moveArmToPosInit(armStops[armSetpointIdx], runtime.seconds());
            }
        }

        public void lowerArmOneStop() {
            setArmRunMode(Motor.RunMode.PositionControl);
            armMotor.setPositionCoefficient(positionCoefficient);
            armMotor.setPositionTolerance(positionTolerance);
            lowerArmToNextStop();
        }

        public void raiseArmOneStop() {
            setArmRunMode(Motor.RunMode.PositionControl);
            armMotor.setPositionCoefficient(positionCoefficient);
            armMotor.setPositionTolerance(positionTolerance);
            raiseArmToNextStop();
        }

        public void moveArmManually(double netTrigger) {
            // have to exceed the threshold to change the run mode
            if (armRunMode != Motor.RunMode.PositionControl || Math.abs(netTrigger) > 0.1) {
                netTrigger = lowerLimit ? Math.max(netTrigger, 0.0) :
                        armMotor.getCurrentPosition() > armMax ? Math.min(netTrigger, 0.0) :
                                netTrigger;

                double cubed = netTrigger * netTrigger * netTrigger;

                this.setArmRunMode(Motor.RunMode.RawPower);

                // this ensures that the next time it switches to positional, it will base it on current position
                armSetpointIdx = -1;

                double armPower = armExpo * cubed + (1.0 - armExpo) * netTrigger;
                double pos = armMotor.getCurrentPosition();

                armMotor.set(pos < closeToZero && armPower < 0.0 ?
                        Math.max(armPower, noWakeSpeed) :
                        pos > armMax - closeToZero ?
                                Math.min(armPower, -noWakeSpeed) :
                                armPower);
            }
        }

        public CancelableAction lowerArmToLimit() {
            return new LowerArmToLimit();
        }

        private class LowerArmToLimit extends CancelableAction {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (touchSensor.isPressed()) {
                    armMotor.set(0);
                    armMotor.resetEncoder();
                    return false;
                }

                if (isCanceled()) {
                    armMotor.set(0);
                    return false;
                }

                if (!initialized) {
                    if (armMovementAction != null) armMovementAction.cancel();
                    armMovementAction = this;
                    armMotor.setRunMode(Motor.RunMode.RawPower);
                    armMotor.set(-0.375);
                    initialized = true;
                }

                packet.put("arm pos", armMotor.getCurrentPosition());

                return true;
            }
        }

        public CancelableAction moveArmToPositionAction(int pos, String stepName, boolean autoWrist) {
            return new MoveArmToPositionAction(pos, stepName, autoWrist);
        }

        public CancelableAction moveArmToPositionAction(int pos, String stepName) {
            return moveArmToPositionAction(pos, stepName, false);
        }

        public CancelableAction moveArmToPositionAction(int pos) {
            return moveArmToPositionAction(pos, "", false);
        }

        public CancelableAction moveArmToStopAction(int stop, boolean autoWrist) {
            return moveArmToPositionAction(armStops[stop], "", autoWrist);
        }

        public CancelableAction moveArmToStopAction(int stop) {
            return moveArmToPositionAction(armStops[stop], "", false);
        }

        public CancelableAction moveArmToStopAction(int stop, String stepName) {
            return moveArmToPositionAction(armStops[stop], stepName);
        }

//        public Action applyAutoWristAction() {
//            return new InstantAction(() -> wrist.setPosition(wristServoValue(armMotor.getCurrentPosition())));
//        }

        private class MoveArmToPositionAction extends CancelableAction {
            private boolean initialized = false;
            private final int targetPos;
            private int rampTarget;
            private boolean rampComplete = false;
            private int armStart;
            private double rampTime;
            private double beginTs;

            private final String stepName;
            private final boolean autoWrist;

            private double now() {
                return System.nanoTime() * 1e-9;
            }

            private boolean softCanceled = false;

            public void softCancel() {
                softCanceled = true;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (isCanceled()) {
                    armMotor.set(0);
                    armMovementAction = null;
                    return false;
                }

                if (softCanceled) {
                    armMovementAction = null;
                    return false;
                }

                if (!initialized) {
                    if (armMovementAction != null) armMovementAction.cancel();
                    armMovementAction = this;
                    armStart = armMotor.getCurrentPosition();
                    packet.put(stepName + "initial position", armStart);
                    this.rampTime = Math.abs(armStart - targetPos) / rampSlope;
                    armMotor.setTelemetry(packet, stepName);
                    setArmRunMode(Motor.RunMode.PositionControl);
                    packet.put(stepName + "target pos", targetPos);
                    beginTs = now();
                    armMotor.moveArmToPosInit(Math.min(armMax, Math.max(0, targetPos)), runtime.seconds());
                    initialized = true;
                }


//                double elapsed = now() - beginTs;
//                rampTarget = elapsed >= rampTime
//                        ? targetPos
//                        : armStart + (targetPos < armStart ? -1 : 1) * (125 + (int) (elapsed * rampSlope));
//
//                packet.put(stepName + "rampTarget", rampTarget);
//                if (!rampComplete) {
//                    rampComplete = rampTarget == targetPos;
//                    armMotor.setTargetPosition(rampTarget);
//                }


                if (touchSensor.isPressed()) {
                    packet.put(stepName + "at limit", true);
                    armMotor.resetEncoder();
                } else {
                    packet.put(stepName + "at limit", false);
                }

                boolean stillRunning = armMotor.moveArmToPosLoop(runtime.seconds());
                packet.put(stepName + "in profile", stillRunning);
//                if (autoWrist){
//                    wrist.setPosition(wristServoValue(armMotor.getCurrentPosition()));
//                }
                if (!stillRunning) {
                    armMovementAction = null;
                }
                return stillRunning;


                //packet.put(stepName + "arm pos", armMotor.getCurrentPosition());

                /*
                if (armMotor.atTargetPosition()) {
                    packet.put(stepName + "at target", true);
                    armMotor.set(0.0);
                    if (autoWrist) {
//                        wrist.setPosition(wristServoValue(armMotor.getCurrentPosition()));
                    }
                    armMovementAction = null;
                    return false;
                } else {
                    packet.put(stepName + "at target", false);
                    armMotor.set(1.0);
                    if (autoWrist) {
//                        wrist.setPosition(wristServoValue(armMotor.getCurrentPosition()));
                    }
                    return true;
                }
                 */
            }

            public MoveArmToPositionAction(int targetPos, String stepName, boolean autoWrist) {
                if (targetPos < 0) {
                    throw new InvalidParameterException("target cannot be negative");
                }
                this.targetPos = targetPos;
                this.stepName = stepName == "" ? "" : stepName + " ";
                this.autoWrist = autoWrist;
            }
        }

        private Action moveGripperAction(double newPosition, int timeMs) {
            if (gripperPosition != newPosition) {
                gripperPosition = newPosition;
                return new SequentialAction(
//                        new InstantAction(() -> gripper.setPosition(newPosition)),
                        new SleepAction(timeMs / 1000.0)
                );
            } else {
                return new NullAction();
            }
        }

        public Action gripperCloseAction() {
            return moveGripperAction(gripperClosedPosition, gripperCloseTime);
        }

        public Action gripperOpenAction() {
            return moveGripperAction(gripperOpenPosition, gripperOpenTime);
        }

        public Action setWristPosition(double pos) {
            if (wristPosition != pos) {
                wristPosition = pos;
                return new SequentialAction(
//                        new InstantAction(() -> wrist.setPosition(pos)),
                        new SleepAction(350 / 1000.0)
                );
            } else {
                return new NullAction();
            }
        }

        public Action setWristTuckedUp() {
            return setWristPosition(wristTuckedUp);
        }

        public Action setWristFlatZero() {
            return setWristPosition(wristFlatZero);
        }

        public void loop() {
            int armTicks = armMotor.getCurrentPosition();
            if (autoWrist) {
                wristPosition = wristServoValue(armTicks);
            }

//            gripper.setPosition(armTicks < armMaxFlat && gripperOpenTimer.isTimerOn() && !gripperOpenTimer.done()
//                    ? gripperClosedPosition
//                    : gripperPosition);
//            wrist.setPosition(wristPosition);

            telemetry.addData("arm: ", armMotor.getCurrentPosition());
            telemetry.addData("timer: ", gripperCloseTimer.elapsedTime());

            lowerLimit = touchSensor.isPressed();

            telemetry.addData("lower limit: ", lowerLimit);

            if (!armEncoderWasReset) {
                armEncoderWasReset = lowerLimit;
            }

            if (getArmRunMode() == Motor.RunMode.PositionControl) {
                armMotor.moveArmToPosLoop(runtime.seconds());
            }

            /*
            if (armRunMode == Motor.RunMode.PositionControl) {
                armMotor.setPositionPI(positionCoefficient, positionIntegralCoeff);
                armMotor.setPositionTolerance(positionTolerance);
                double armPower = armMotor.atTargetPosition() ||
                        (lowerLimit && armMotor.getCurrentPosition() > armStops[armSetpointIdx]) ? 0.0 : 1.0;
                armMotor.set(armPower);
                // if it's already at the lower limit and the arm is trying to go down, stop it
            }
             */

            if (lowerLimit) {
                armMotor.resetEncoder();
            }
        }
    }

    public boolean isArmEncoderReset() {
        return armEncoderWasReset;
    }

    public static double boostSpeed = 1.0;
    public static double normalSpeed = .6;

    private FtcLibMecanumDrive ftcLibMecanumDrive;

    public FtcLibMecanumDrive ftcLibMecanumDrive() {
        if (ftcLibMecanumDrive == null) {
            ftcLibMecanumDrive = new FtcLibMecanumDrive();
            //ftcLibMecanumDrive.init(PoseStorage.currentPose.heading.log());
            ftcLibMecanumDrive.init(0);
        }
        return ftcLibMecanumDrive;
    }

    public class FtcLibMecanumDrive {
        private Motor rightFront, leftFront, rightBack, leftBack;
        private MecanumDrive drive;
        private RevIMU imu;
        private double headingOffset;

        public void init(double headingOffset) {
            // Initialize drive motors
            rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
            leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
            rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);
            leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);

            // Invert the drive motors
            rightFront.setInverted(true);
            leftFront.setInverted(true);
            rightBack.setInverted(true);
            leftBack.setInverted(true);

            // Initialize the drivetrain
            drive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);

            imu = new RevIMU(hardwareMap);
            imu.init();
            this.headingOffset = headingOffset;
        }

        public void resetHeading() {
            imu.reset();
        }

        public void loop(boolean boost, boolean fieldCentric, double strafeInput, double forwardInput, double turnInput) {
            double driveSpeed = boost ? boostSpeed : normalSpeed;

            if (!fieldCentric) {
                drive.driveRobotCentric(
                        strafeInput * driveSpeed,
                        forwardInput * driveSpeed,
                        turnInput * driveSpeed * .7
                );
            } else {
                drive.driveFieldCentric(
                        strafeInput * driveSpeed,
                        forwardInput * driveSpeed,
                        turnInput * driveSpeed * .7,
                        imu.getRotation2d().getDegrees() - headingOffset
                );
            }

            // Show the elapsed game time and wheel power.
            telemetry.addLine("========== Encoders ==========");
            telemetry.addData("Par:   ", rightFront.getCurrentPosition());
            telemetry.addData("Perp:  ", leftFront.getCurrentPosition());
        }
    }

    public static double launcherOpenPosition = 0.7;
    public static double launcherClosedPosition = 1;
    public static double launcherPosition = launcherClosedPosition;

    private DroneLauncher droneLauncher;

    public DroneLauncher droneLauncher() {
        if (droneLauncher == null) {
            droneLauncher = new DroneLauncher();
            droneLauncher.init();
        }
        return droneLauncher;
    }

    public class DroneLauncher {
        private Servo droneLaunch;

        public void init() {
            droneLaunch = hardwareMap.servo.get("droneLaunch");
        }

        public void loop() {
            droneLaunch.setPosition(launcherPosition);
        }

        public void launchDrone() {
            launcherPosition = launcherOpenPosition;
        }

        public void holdDrone() {
            launcherPosition = launcherClosedPosition;
        }

        public Action stowLauncherArmAction() {
            return new InstantAction(() -> droneLaunch.setPosition(launcherClosedPosition));
        }
    }

    private Winch winch;

    public Winch winch() {
        if (winch == null) {
            winch = new Winch();
        }

        return winch;
    }

    public class Winch {
//        private Motor winchMotor;

        public Winch() {
//            winchMotor = new Motor(hardwareMap, "winchMotor", Motor.GoBILDA.RPM_117);
//            winchMotor.setRunMode(Motor.RunMode.RawPower);
        }

        public void windUp() {
//            winchMotor.set(0.3);
        }

        public void windDown() {
//            winchMotor.set(-0.3);
        }

        public void stop() {
//            winchMotor.set(0);
        }

        public void setPower(double raw) {
//            winchMotor.set(raw);
        }

//        public void writeTelemetry() {
//            telemetry.addData("winchPosition: ", winchMotor.getCurrentPosition());
//        }
    }
}
