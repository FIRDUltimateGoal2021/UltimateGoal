package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleSystem {
    public final LinearOpMode opMode;
    public final Servo servo;

    public WobbleSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        servo = opMode.hardwareMap.get(Servo.class, "WobbleServo");
    }
}
