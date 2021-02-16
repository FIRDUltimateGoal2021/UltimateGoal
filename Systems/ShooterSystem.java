package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ShooterSystem {
    public final LinearOpMode opMode;
    public final Servo angleServo, ammo;
    public final DcMotor motor;
    public double currentHorizontalAngle;

    public ShooterSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        motor = opMode.hardwareMap.get(DcMotor.class, "ShootingMotor");
        angleServo = opMode.hardwareMap.get(Servo.class, "AngleServo");
        ammo = opMode.hardwareMap.get(Servo.class, "Ammunition");
    }

    /**
     * @param newHorizontalAngle the angle of the shooter above the horizontal plane in degrees.
     */
    public void changeAngle(double newHorizontalAngle) {
        currentHorizontalAngle = newHorizontalAngle;

        // in radians
        final double servoAngle0 = 0;
        final double armToShooterRatio = 1.024;

        final double newServoAngle = Math.asin(
                Math.sin(servoAngle0) + armToShooterRatio * Math.sin(Math.toRadians(newHorizontalAngle))
        );
        angleServo.setPosition(newServoAngle / Math.PI);
    }

    public void load() {
        ammo.setPosition(0.5);
    }

    public void shoot() {
        final double initialPosition = 0;
        ammo.setPosition(initialPosition);
    }

    public void off() {
        motor.setPower(0);
    }

    public void on() {
        motor.setPower(1);
    }
}
