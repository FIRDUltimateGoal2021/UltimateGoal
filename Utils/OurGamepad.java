package org.firstinspires.ftc.teamcode.UltimateGoal.Utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class OurGamepad {
    private final Gamepad gamepad;

    public boolean previousA;
    public boolean previousB;
    public boolean previousX;
    public boolean previousY;

    public boolean previousRb;
    public boolean previousLb;
    public boolean previousRt;
    public boolean previousLt;


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
            case "Rb":
                if (gamepad.right_bumper && !previousRb) {
                    return true;
                }
                break;
            case "Lb":
                if (gamepad.left_bumper && !previousLb) {
                    return true;
                }
                break;
            case "Rt":
                if (gamepad.right_trigger > 0 && !previousRt) {
                    return true;
                }
                break;
            case "Lt":
                if (gamepad.left_trigger > 0 && !previousLt) {
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
        previousRb = gamepad.right_bumper;
        previousLb = gamepad.left_bumper;
        previousRt = (gamepad.right_trigger > 0);
        previousLt = (gamepad.left_trigger > 0);
    }
}

