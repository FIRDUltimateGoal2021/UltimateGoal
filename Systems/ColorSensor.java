//package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;
//
//import android.graphics.Color;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//
//public class ColorSensor {
//    public enum ColorEnum {
//        RED,
//        BLUE,
//        WHITE,
//        DORIAN
//    }
//
//    LinearOpMode opMode;
//    NormalizedColorSensor normalizedColorSensor;
//
//    public ColorSensor(LinearOpMode opMode) {
//        this.opMode = opMode;
//        normalizedColorSensor = opMode.hardwareMap.get(NormalizedColorSensor.class,
//                "ColorSensor");
//    }
//
//    public ColorEnum getColor() {
//        final float[] hsvValues = new float[3];
//
//        NormalizedRGBA colors = normalizedColorSensor.getNormalizedColors();
//        Color.colorToHSV(colors.toColor(), hsvValues);
//        opMode.telemetry.addLine()
//                .addData("HSV: ", hsvValues[0])
//                .addData(" ", hsvValues[1])
//                .addData(" ", hsvValues[2]);
//
//        if (hsvValues[1] > 0.5 && hsvValues[2] > 0.5) {
//            if (hsvValues[0] < 260 && hsvValues[0] > 200) {
//                opMode.telemetry.addLine("color: blue");
//                opMode.telemetry.update();
//                return ColorEnum.BLUE;
//            }
//            if ((hsvValues[0] > 0 || hsvValues[0] > 340)) {
//                opMode.telemetry.addLine("color: red");
//                opMode.telemetry.update();
//                return ColorEnum.RED;
//            }
//        }
//
//        if (hsvValues[1] < 0.2 && hsvValues[2] > 0.1) {
//            opMode.telemetry.addLine("color: white");
//            opMode.telemetry.update();
//            return ColorEnum.WHITE;
//        }
//        opMode.telemetry.addLine("color: dorian");
//        opMode.telemetry.update();
//        return ColorEnum.DORIAN;
//    }
//}
