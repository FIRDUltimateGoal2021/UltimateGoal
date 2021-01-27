package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ShooterSystem {
    private final LinearOpMode opMode;
    private final DcMotor motor;
    private final Servo angleServo, ammo;
    private final double ammoRestAngle = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public ShooterSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        motor = opMode.hardwareMap.get(DcMotor.class, "ShootTheShitOutOfEveryoneMotor");
        angleServo = opMode.hardwareMap.get(Servo.class, "AngleServo");
        ammo = opMode.hardwareMap.get(Servo.class, "Ammunition");
    }

    /**
     * @param newHorizontalAngle the angle of the shooter above the horizontal plane in degrees.
     */
    public void changeAngle(double newHorizontalAngle) {
        // in degrees
        final double servoAngle0 = 0;

        // in whatever, probably mm
        final double armLength = 1;
        final double shooterLength = 1;

        final double newServoAngle = Math.asin(Math.sin(3.14 / 180 * servoAngle0)
                + shooterLength / armLength * Math.sin(3.14 / 180 * newHorizontalAngle));
        angleServo.setPosition(newServoAngle / 3.14);
    }

    public void loadAmmo() {
        ammo.setPosition(ammoRestAngle + 0.5);
    }

    public void releaseAmmo() {
        ammo.setPosition(ammoRestAngle);
        motor.setPower(1);

        timer.reset();
        while (timer.time() < 5) {
            System.out.println("shooting their goddamn arses");
        }
        motor.setPower(0);
    }

//    public void stopShooting() {
//        motor.setPower(0);
//    }
}
