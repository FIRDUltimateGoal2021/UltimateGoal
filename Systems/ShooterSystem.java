package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class ShooterSystem {
    LinearOpMode opMode;
    DcMotor motor;
    Servo angleServo;
    Servo ammo;
    double ammoRestAngle = 0;

    public ShooterSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        motor = opMode.hardwareMap.get(DcMotor.class, "ShootTheShitOutOfEveryone");
        angleServo = opMode.hardwareMap.get(Servo.class, "AngleServo");
        ammo = opMode.hardwareMap.get(Servo.class, "Ammunition");
    }

    /**
     * @param newHorizontalAngle the angle of the shooter above the horizontal plane in degrees.
     */
    public void changeAngle(double newHorizontalAngle) {
        // in degrees
        double servoAngle0 = 0;

        // in whatever, probably mm
        double armLength = 1;
        double shooterLength = 1;

        double newServoAngle = Math.asin(Math.sin(3.14 / 180 * servoAngle0)
                + shooterLength / armLength * Math.sin(3.14 / 180 * newHorizontalAngle));
        angleServo.setPosition(newServoAngle / 3.14);
    }

    public void loadAmmo() {
        ammo.setPosition(ammoRestAngle + 0.5);
    }

    public void releaseAmmo() {
        ammo.setPosition(ammoRestAngle);
        motor.setPower(1);
        // possibly add timer for motor
    }

    public void stopShooting() {
        motor.setPower(0);
    }
}
