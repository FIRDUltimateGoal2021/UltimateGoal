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
    public boolean loaded = false;

    public ShootingSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        motor = opMode.hardwareMap.get(DcMotor.class, "ShootingMotor");
        angleServo = opMode.hardwareMap.get(Servo.class, "AngleServo");
        load();
    }

    public void shoot() {
        changeAngle(-55);
        loaded = false;
    }

    public void load() {
        changeAngle(-40);
        loaded = true;
    }

    public void shootLoad() {
        if (loaded) {
            shoot();
        } else {
            load();
        }
    }

    public void changeAngle(double newAngle) {
        double a0 = 90;
        currentAngle = newAngle;
        angleServo.setPosition((currentAngle + a0) / 180);
        opMode.telemetry.addData("Current Servo Angle: ", newAngle);
    }

    public void toggle() {
        if (isOn) {
            off();
        } else {
            on();
        }
    }

    public void spit() {
        if (isOn) {
            off();
        } else {
            motor.setPower(1);
            isOn = true;
        }

    }

    public void off() {
        motor.setPower(0);
        isOn = false;
    }

    public void on() {
        motor.setPower(-1);
        isOn = true;
    }
}
