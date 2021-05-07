package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ShootingSystem {
    public final LinearOpMode opMode;
    public final Servo angleServo;
    public final DcMotor motor;
    public double currentAngle = 0;
    public boolean isOn = false;

    public ShootingSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        motor = opMode.hardwareMap.get(DcMotor.class, "ShootingMotor");
        angleServo = opMode.hardwareMap.get(Servo.class, "AngleServo");
        changeAngle(0);
    }

    public void shoot() {
        changeAngle(75);
    }

    public void load() {
        changeAngle(30);
    }

    public void changeAngle(double newAngle) {
        double a0 = 90;
        currentAngle = a0 + newAngle;
        angleServo.setPosition(currentAngle / 180);
    }

    public void toggle() {
        if (isOn) {
            off();
        } else {
            on();
        }
    }

    public void off() {
        motor.setPower(0);
        isOn = false;
    }

    public void on() {
        motor.setPower(1);
        isOn = true;
    }
}
