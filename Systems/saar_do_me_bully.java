package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class saar_do_me_bully {
   public enum sss{
        RED,
        BLUE,
        WHITE,
        DORIAN
    }
    NormalizedColorSensor gargasaar;
   public saar_do_me_bully(NormalizedColorSensor s4ar){

       gargasaar= s4ar;
    }

    public sss getcolor(){
        final float[] hsvValues = new float[3];
        NormalizedRGBA colors = gargasaar.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        if (hsvValues[1]>0.5&&hsvValues[2]>0.5) {
            if (hsvValues[0] < 260 && hsvValues[0] > 200) {
                return sss.BLUE;
            }
            if ((hsvValues[0] < 30 || hsvValues[0] > 340)) {
                return sss.RED;
            }
        }
        if(hsvValues[1]<0.25&&hsvValues[2]>0.25){
                return sss.WHITE;
        }
        return sss.DORIAN;
    }

}
