package frc.robot.lib.input;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.GenericHID;

public class HIDEventHandler {
    
    private GenericHID controller;

    // An array that stores whether a button has just received a press on the last update.
    public boolean[] buttonEvents;
    public int buttonCount;
    
    public HIDEventHandler (GenericHID c) {
        controller = c;
        buttonCount = c.getButtonCount();
        buttonEvents = new boolean[buttonCount];
    }

    public void update () {
        for (int b = 0; b < buttonCount; b++) {
            if (controller.getRawButtonPressed(b)) {
                if (!buttonEvents[b]) {
                    buttonEvents[b] = true;
                }
                else {
                    buttonEvents[b] = false;
                }
            }
            else {
                buttonEvents[b] = false;
            }
        }
    }

    public boolean getRawButtonPressEvent (int b) {
        return buttonEvents[b];
    } 

}
