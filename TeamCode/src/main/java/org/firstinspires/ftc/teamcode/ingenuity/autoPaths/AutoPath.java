package org.firstinspires.ftc.teamcode.ingenuity.autoPaths;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PhaseTwoBot;

enum StartingPosition {
    FRONT_BLUE,
    FRONT_RED,
    BACK_BLUE,
    BACK_RED
}

@Config
public abstract class AutoPath {

    public static double initPauseBack = 0;
    public static double initPauseFront = 5.0;
    public static double initXFront = -36;
    public static double initXBack = 12;
    public static double initYRed = -61;
    public static double initYBlue = 61;
    public static double initAngleRed = 90;
    public static double initAngleBlue = 270;

    public static double armDelayFront = 0.15;
    public static double armDelayBack = 0.15;
    public static double parkingX = 58;
    public static double parkingYFrontRed = -8;
    public static double parkingYFrontBlue = 8;
    public static double parkingYBackRed = -56;
    public static double parkingYBackBlue = 56;

    protected static double initPause;
    protected static double initX;
    protected static double initY;
    protected static double initAngle;
    protected static double parkingY;
    protected static double armDelay;
    protected static int backDelivery = Math.min(PhaseTwoBot.armMax, 2110);


    protected final StartingPosition startingPosition;

    protected final PhaseTwoBot bot;
    protected final OpMode opMode;
    protected final MecanumDrive drive;

    protected final TranslationalVelConstraint slow = new TranslationalVelConstraint(15);
    protected final TranslationalVelConstraint mediumSpeed = new TranslationalVelConstraint(25);

    protected AutoPath(StartingPosition startingPosition, OpMode newOpMode, PhaseTwoBot newBot, MecanumDrive newDrive) {
        opMode = newOpMode;
        bot = newBot;
        drive = newDrive;

        this.startingPosition = startingPosition;
        switch (startingPosition) {
            case FRONT_BLUE:
                initPause = initPauseFront;
                initX = initXFront;
                initY = initYBlue;
                parkingY = parkingYFrontBlue;
                initAngle = initAngleBlue;
                armDelay = armDelayFront;
                break;
            case FRONT_RED:
                initPause = initPauseFront;
                initX = initXFront;
                initY = initYRed;
                parkingY = parkingYFrontRed;
                initAngle = initAngleRed;
                armDelay = armDelayFront;
                break;
            case BACK_BLUE:
                initPause = initPauseBack;
                initX = initXBack;
                initY = initYBlue;
                parkingY = parkingYBackBlue;
                initAngle = initAngleBlue;
                armDelay = armDelayBack;
                break;
            case BACK_RED:
                initPause = initPauseBack;
                initX = initXBack;
                initY = initYRed;
                parkingY = parkingYBackRed;
                initAngle = initAngleRed;
                armDelay = armDelayBack;
                break;
        }
    }

    public static double reverseAngle(double degrees) {
        return (degrees + 180.0) % 360.0;
    }

    public abstract void runAutoPath();

    public void waitForTime(double time) {
        double startTime = opMode.getRuntime();
        while (IsTimeUp(startTime, time)) {
        }
    }

    public boolean IsTimeUp(double startTime, double runTime) {
        return opMode.getRuntime() < startTime + runTime;
    } // From Owen
}
