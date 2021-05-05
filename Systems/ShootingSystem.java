package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ShootingSystem {
    public final LinearOpMode opMode;
    public final Servo angleServo;
    public final DcMotor motor;

    public ShootingSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        motor = opMode.hardwareMap.get(DcMotor.class, "ShootingMotor");
        angleServo = opMode.hardwareMap.get(Servo.class, "AngleServo");
    }

    public void shoot() {
        changeAngle(30);
    }

    public void changeAngle(double newHorizontalAngle) {
        double a0 = 30;
        angleServo.setPosition((a0 + newHorizontalAngle) / 180);
    }

    public void off() {
        motor.setPower(0);
    }

    public void on() {
        motor.setPower(1);
    }
}
