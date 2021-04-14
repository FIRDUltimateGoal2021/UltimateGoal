package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Function;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


public class DrivingSystem {
    Orientation angles;
    BNO055IMU imu;
    LinearOpMode opMode;
    DcMotor leftMotor;
    DcMotor rightMotor;
    double v;
    double r;
    public DrivingSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        leftMotor = opMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = opMode.hardwareMap.get(DcMotor.class, "right_drive");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
public void moveteta (double teta){
        double k;
        if (teta <0)
        {
            k=-1;
        }
        else {
            k= 1;
        }
    double dt = 1 / 60f;
    ElapsedTime timer = new ElapsedTime();
    angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        for (int g=0;angles.firstAngle<teta;g+= dt) {


            timer.reset();

            driveByJoystick(v*k,-v*k );
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
public void easy (double teta1, double teta2, double dx, double dy){
moveteta(teta1);
    double rr =dx*dx+dy*dy;
    ElapsedTime timer = new ElapsedTime();

    double dt = 1 / 60f;
    for (double t = 0; t <rr/v; t += dt) {
        timer.reset();


        driveByJoystick(1, 1);
        if (timer.seconds() < dt) {
            opMode.sleep((long) ((dt - timer.seconds()) * 1000));
        }
    }
    stöp();

    moveteta(teta2);
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

    public void drive(double speed){
        rightMotor.setPower(speed);
        leftMotor.setPower(speed);
    }
    public void stopDriving(){
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
}
