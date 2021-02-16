package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class CollectionSystem {
    public final LinearOpMode opMode;
    public final DcMotor motor;

    // TODO: 09/02/2021 initialize motor speed and maybe consider adding also an on off ability as well as toggle 
    public CollectionSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        this.motor = opMode.hardwareMap.get(DcMotor.class, "CollectionMotor");
    }

    public void toggle() {
        if (motor.getPower() == 1) {
            motor.setPower(0);
        } else {
            motor.setPower(1);
        }
    }
}
