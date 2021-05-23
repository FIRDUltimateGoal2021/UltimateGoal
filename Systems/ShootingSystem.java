package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class ShootingSystem {
    public final LinearOpMode opMode;
    public final Servo angleServo;
    public final Servo angleServo2;
    public final DcMotorEx motor;
    public double currentAngle = 0;
    public boolean isOn = false;
    public boolean loaded = false;

    public ShootingSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        motor       = opMode.hardwareMap.get(DcMotorEx.class, "ShootingMotor");
        angleServo  = opMode.hardwareMap.get(Servo.class, "AngleServoRight");
        angleServo2 = opMode.hardwareMap.get(Servo.class, "AngleServoLeft");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shoot() {
        changeAngle(28);
        loaded = false;
    }

    public void load() {
        changeAngle(0);
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
        currentAngle = newAngle;
        angleServo.setPosition(1 - (currentAngle) / 180);
        angleServo2.setPosition((currentAngle) / 180);
        opMode.telemetry.addData("Current Servo Angle: ", newAngle);
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
        motor.setVelocity(14400);
        motor.setPower(-1);
        isOn = true;
    }

    public void smallSpeed(){
        motor.setVelocity(28800);
    }

    public void mediumSpeed(){
        motor.setVelocity(36000);
    }

    public void highSpeed(){
        motor.setVelocity(43200);
    }


}
