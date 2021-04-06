package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;


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

    /**
     * not approved by S4AR.inc
     * DO NOT USE
     */
    @Deprecated
    public void shootfast(Position target,int tt) {
        on();
       double isaekai = Math.sqrt(Math.pow(target.x,2)+Math.pow(target.y,2));
       double ang =Math.atan2(isaekai,target.z);
       changeAngle(ang);
       for (int ttt=0;ttt<tt;ttt++) {


           shoot();
           opMode.sleep(200);
           load();
       }
    }

    /**
     * shoots a ring at target position
     * S4AR.inc certificate of approval
     * @param target the position of the target relative to the robot
     */
    public void shootfaster(Position target) {
        on();
        double isaekai = Math.sqrt(Math.pow(target.x,2)+Math.pow(target.y,2));
        double ang =Math.atan2(isaekai,target.z);
        changeAngle(ang);
        shoot();
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
