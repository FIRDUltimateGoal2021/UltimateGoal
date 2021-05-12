package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Function;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.UltimateGoal.Utils.PIDController;


public class DrivingSystem {
    Orientation  angles;
    BNO055IMU    imu;
    LinearOpMode opMode;
    DcMotor      leftMotor;
    DcMotor      rightMotor;
    double       v = 0.51;
    double       r = 0.16;
    PIDController pid = new PIDController(0.01, 0.0, 0.0);

    double globalAng;

    boolean     startedStopping = false;
    ElapsedTime timer           = new ElapsedTime();

    public DrivingSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        leftMotor   = opMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor  = opMode.hardwareMap.get(DcMotor.class, "right_drive");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        globalAng = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void movetheta(double theta) {
        double k;
        if (theta < 0) {
            k = -1;
        } else {
            k = 1;
        }
        double dt = 1 / 60f;
        ElapsedTime timer = new ElapsedTime();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double firstangels = angles.firstAngle;
        if (firstangels + theta > 180) {
            // firstangels
        }
        for (; angles.firstAngle < theta + firstangels && opMode.opModeIsActive();
             angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)) {
            opMode.telemetry.addData("זווית נוכחית", angles);
            opMode.telemetry.addData("זווית מטרה", theta + firstangels);
            opMode.telemetry.update();
            timer.reset();

            driveByJoystick(v * k, -v * k);
            if (timer.seconds() < dt) {
                opMode.sleep((long) ((dt - timer.seconds()) * 1000));
            }
        }
        stöp();
    }

    public void driveByJoystick(double horizontal, double vertical) {
        double left = vertical - horizontal;
        double right = vertical + horizontal;
        if (Math.abs(left) > 1 || Math.abs(right) > 1) {
            left  = left * Math.max(Math.abs(left), Math.abs(right));
            right = right * Math.max(Math.abs(left), Math.abs(right));
        }
        leftMotor.setPower(-left);
        rightMotor.setPower(-right);
    }

    public void driveAutonomously(Position[] way) {
        Function<Double, Position> func = new Function<Double, Position>() {
            @Override
            public Position apply(Double aDouble) {
                return new Position(DistanceUnit.METER, 0, 0, 0, 0);
            }
        };
        for (Position pos : way) {
            //
        }
    }

    public void easy(double teta1, double teta2, double dx, double dy) {
        movetheta(teta1);
        double rr = dx * dx + dy * dy;
        ElapsedTime timer = new ElapsedTime();

        double dt = 1 / 60f;
        for (double t = 0; t < rr / v; t += dt) {
            timer.reset();

            driveByJoystick(1, 1);
            if (timer.seconds() < dt) {
                opMode.sleep((long) ((dt - timer.seconds()) * 1000));
            }
        }
        stöp();

        movetheta(teta2);
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

            before  = func.apply(t - dt);
            current = func.apply(t);
            after   = func.apply(t + dt);

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

    public void betterStöp() {
        rightMotor.setPower(-1);
        leftMotor.setPower(1);
        if (!startedStopping) {
            timer           = new ElapsedTime();
            startedStopping = true;
        } else if (timer.seconds() >= 0.5) {
            startedStopping = false;
            stöp();
        }
    }


    //nati's code

    public void turn(double ang) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double destination = angles.firstAngle + ang;

        if (destination > 180)
            destination -= 360;
        else if (destination < -180)
            destination += 360;

        double currentTurningAngle = angles.firstAngle;

        int turnSide = -1;
        if (currentTurningAngle > destination)
            turnSide = 1;

        globalAng = destination;

        if (currentTurningAngle > destination) {
            while (currentTurningAngle > destination) {
                driveByJoystick(0, turnSide * 0.5);
                currentTurningAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                opMode.telemetry.addData("destination", destination);
                opMode.telemetry.addData("currentAngle", currentTurningAngle);
                opMode.telemetry.update();
            }
        } else {
            while (currentTurningAngle < destination) {
                driveByJoystick(0, turnSide * 0.5);
                currentTurningAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                opMode.telemetry.addData("destination", destination);
                opMode.telemetry.addData("currentAngle", currentTurningAngle);
                opMode.telemetry.update();
            }
        }
        stöp();
    }

    public void driveForward(double time, double speed, double devideBy) {
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() < time) {
            double correction = angleCorrection()/devideBy;
            driveByJoystick(speed, correction);
        }
        stöp();
    }

    public void driveForwardNoCorrection(double time, double speed) {
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() < time) {
            driveByJoystick(speed, 0);
        }
        stöp();
    }

    private double angleCorrection() {
        double ang = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (ang > 90 && globalAng < -90)
            return ((180 - ang) - (-180 - globalAng));
        if (ang < -90 && globalAng > 90)
            return ((-180 - globalAng) - (180 - ang));
        return (ang - globalAng);
    }

    public void printXYZ() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        opMode.telemetry.addData("1", angles.firstAngle);
        opMode.telemetry.addData("2", angles.secondAngle);
        opMode.telemetry.addData("3", angles.thirdAngle);
        opMode.telemetry.update();
    }

}
