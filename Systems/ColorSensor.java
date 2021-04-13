package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSensor {
    public enum ColorEnum {
        RED,
        BLUE,
        WHITE,
        DORIAN
    }

    LinearOpMode opMode;
    NormalizedColorSensor normalizedColorSensor;

    public ColorSensor(LinearOpMode opMode) {
        this.opMode = opMode;
        normalizedColorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
    }

    public ColorEnum getColor() {
        final float[] hsvValues = new float[3];

        NormalizedRGBA colors = normalizedColorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        opMode.telemetry.addLine()
                .addData("HSV: ", hsvValues[0])
                .addData(" ", hsvValues[1])
                .addData(" ", hsvValues[2]);

        if (hsvValues[1] > 0.5 && hsvValues[2] > 0.5) {
            if (hsvValues[0] < 260 && hsvValues[0] > 200) {
                return ColorEnum.BLUE;
            }
            if ((hsvValues[0] > 0 || hsvValues[0] > 340)) {
                return ColorEnum.RED;
            }
        }

        if (hsvValues[1] < 0.5 && hsvValues[2] > 0.5) {
            return ColorEnum.WHITE;
        }
        return ColorEnum.DORIAN;
    }
}
