package org.firstinspires.ftc.teamcode.UltimateGoal.Utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class OurGamepad {
    private final Gamepad gamepad;
    public boolean previousA;
    public boolean previousB;
    public boolean previousX;
    public boolean previousY;


    public OurGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public boolean buttonPress(String button) {
        switch (button) {
            case "a":
                if (gamepad.a && !previousA) {
                    return true;
                }
                break;
            case "b":
                if (gamepad.b && !previousB) {
                    return true;
                }
                break;
            case "x":
                if (gamepad.x && !previousX) {
                    return true;
                }
                break;
            case "y":
                if (gamepad.y && !previousY) {
                    return true;
                }
                break;
        }
        return false;
    }

    public void update() {
        previousA = gamepad.a;
        previousB = gamepad.b;
        previousX = gamepad.x;
        previousY = gamepad.y;
    }
}

