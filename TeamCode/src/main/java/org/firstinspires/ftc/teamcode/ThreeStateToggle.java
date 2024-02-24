package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class ThreeStateToggle extends ButtonReader {
    private int currToggleState;

    /**
     * The constructor that uses the gamepad and button to refer to a certain state toggler.
     *
     * @param gamepad the gamepad object that contains the buttonn
     * @param button  the button on the oject
     */
    public ThreeStateToggle(GamepadEx gamepad, GamepadKeys.Button button) {
        super(gamepad, button);

        currToggleState = 0;
    }

    /**
     * @return the current state of the toggler
     */
    public int getState() {
        if (wasJustReleased()) {
            currToggleState = (currToggleState + 1) % 3;
        }
        return (currToggleState);
    }
}
