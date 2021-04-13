package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Function;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


public class DrivingSystem {

    LinearOpMode opMode;
    DcMotor leftMotor;
    DcMotor rightMotor;

    public DrivingSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        leftMotor = opMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = opMode.hardwareMap.get(DcMotor.class, "right_drive");
    }

    public void driveByJoystick(double horizontal, double vertical) {
        double left = vertical - horizontal;
        double right = vertical + horizontal;
        if (Math.abs(left) > 1 || Math.abs(right) > 1) {
            left = left * Math.max(Math.abs(left), Math.abs(right));
            right = right * Math.max(Math.abs(left), Math.abs(right));
        }
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

    public void driveAutonomously(Position[] way) {
        Function<Double, Position> func = new Function<Double, Position>() {
            @Override
            public Position apply(Double aDouble) {
                return new Position(DistanceUnit.METER,0,0,0,0);
            }
        };
        for (Position pos : way) {
            //
        }
    }

    public void driveAutonomouslyBetter(
            double t0, double tn, Function<Double, Position> func
    ) {
        ElapsedTime timer = new ElapsedTime();

        double dt = 1 / 60f;
        for (double t = t0; t < tn && opMode.opModeIsActive(); t += dt) {
            timer.reset();

            Position before;
            Position current;
            Position after;

            before = func.apply(t - dt);
            current = func.apply(t);
            after = func.apply(t + dt);

            double vxAfter = (after.x - current.x) / dt;
            double vyAfter = (after.y - current.y) / dt;

            double vxBefore = (current.x - before.x) / dt;
            double vyBefore = (current.y - before.y) / dt;

            double ax = (vxAfter - vxBefore) / dt;
            double ay = (vyAfter - vyBefore) / dt;

            double theta = Math.atan2(vxAfter, vyAfter);
            double aRadial = ay * Math.cos(theta) + ax * Math.sin(theta);
            double aTangent = ay * Math.sin(theta) - ax * Math.cos(theta);

            driveByJoystick(aTangent, aRadial);
            if (timer.seconds() < dt) {
                opMode.sleep((long) ((dt - timer.seconds()) * 1000));
            }
        }
        stöp();
    }

    public void stöp() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void driveForward(double distance) {
    }

    /**
     * @param angle in degrees
     */
    public void rotate(double angle) {
    }
}
