package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class SaarDoMeBully {
    public enum ColorEnum {
        RED,
        BLUE,
        WHITE,
        DORIAN
    }

    NormalizedColorSensor normalizedColorSensor;

    public SaarDoMeBully(NormalizedColorSensor s4ar) {
        normalizedColorSensor = s4ar;
    }

    public ColorEnum getColor() {
        final float[] hsvValues = new float[3];

        NormalizedRGBA colors = normalizedColorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        if (hsvValues[1] > 0.5 && hsvValues[2] > 0.5) {
            if (hsvValues[0] < 260 && hsvValues[0] > 200) {
                return ColorEnum.BLUE;
            }
            if ((hsvValues[0] < 30 || hsvValues[0] > 340)) {
                return ColorEnum.RED;
            }
        }

        if (hsvValues[1] < 0.25 && hsvValues[2] > 0.25) {
            return ColorEnum.WHITE;
        }
        return ColorEnum.DORIAN;
    }
}
