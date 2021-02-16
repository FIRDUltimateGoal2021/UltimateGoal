package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class CollectionSystem {
    public final LinearOpMode opMode;
    public final DcMotor motor;

    public CollectionSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        motor = opMode.hardwareMap.get(DcMotor.class, "CollectionMotor");
        off();
    }

    public void toggle() {
        if (motor.getPower() == 1) {
            off();
        } else {
            on();
        }
    }

    public void off() {
        motor.setPower(0);
    }

    public void on() {
        motor.setPower(1);
    }
}
