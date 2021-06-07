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
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.UltimateGoal.Utils.PIDController;


public class DrivingSystem {

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_MMS = 116;     // For figuring circumference
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MMS * 3.1415);

    Orientation angles;
    BNO055IMU imu;
    LinearOpMode opMode;
    DcMotor leftMotor;
    DcMotor rightMotor;
    double v = 0.51;
    double r = 0.16;


    double globalAng;

    boolean startedStopping = false;
    ElapsedTime timer = new ElapsedTime();

    public DrivingSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        leftMotor = opMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = opMode.hardwareMap.get(DcMotor.class, "right_drive");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        globalAng = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void driveByJoystick(double horizontal, double vertical) {
        double left = vertical + horizontal;
        double right = vertical - horizontal;
        if (Math.abs(left) > 1 || Math.abs(right) > 1) {
            left = left / Math.max(Math.abs(left), Math.abs(right));
            right = right / Math.max(Math.abs(left), Math.abs(right));
        }
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }

    public void stöp() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void betterStöp() {
        rightMotor.setPower(-1);
        leftMotor.setPower(1);
        if (!startedStopping) {
            timer = new ElapsedTime();
            startedStopping = true;
        } else if (timer.seconds() >= 0.5) {
            startedStopping = false;
            stöp();
        }
    }

    public void driveTurn(double turnSpeed) {
        leftMotor.setPower(turnSpeed);
        rightMotor.setPower(turnSpeed);
    }

    public void turn(double changeAng, double forwardSpeed) {
        final double turnSpeed = 0.5;

//        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angle = globalAng;
        double destination = angle + changeAng;

        if (destination > 180)
            destination -= 360;
        else if (destination < -180)
            destination += 360;

        if (destination > 178 || destination < -178) {
            turn180(forwardSpeed, turnSpeed, destination);
        } else {

            double currentTurningAngle = angle;

            if (currentTurningAngle > 90 && destination < -90) {
                while (currentTurningAngle > 0) {
                    driveByJoystick(-forwardSpeed, -turnSpeed);
//                    driveTurn(-turnSpeed);
                    opMode.telemetry.addData("globalAng:", globalAng);
                    opMode.telemetry.addData("destination:", destination);
                    opMode.telemetry.addData("currentAng:", currentTurningAngle);
                    opMode.telemetry.update();
                    currentTurningAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
                while (currentTurningAngle < destination) {
                    driveByJoystick(-forwardSpeed, -turnSpeed);
//                    driveTurn(-turnSpeed);
                    opMode.telemetry.addData("globalAng:", globalAng);
                    opMode.telemetry.addData("destination:", destination);
                    opMode.telemetry.addData("currentAng:", currentTurningAngle);
                    opMode.telemetry.update();
                    currentTurningAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            } else if (currentTurningAngle < -90 && destination > 90) {
                while (currentTurningAngle < 0) {
                    driveByJoystick(-forwardSpeed, turnSpeed);
//                    driveTurn(turnSpeed);
                    opMode.telemetry.addData("globalAng:", globalAng);
                    opMode.telemetry.addData("destination:", destination);
                    opMode.telemetry.addData("currentAng:", currentTurningAngle);
                    opMode.telemetry.update();
                    currentTurningAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
                while (currentTurningAngle > destination) {
                    driveByJoystick(-forwardSpeed, turnSpeed);
//                    driveTurn(turnSpeed);
                    opMode.telemetry.addData("globalAng:", globalAng);
                    opMode.telemetry.addData("destination:", destination);
                    opMode.telemetry.addData("currentAng:", currentTurningAngle);
                    opMode.telemetry.update();
                    currentTurningAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            } else if (currentTurningAngle > destination) {
                while (currentTurningAngle > destination) {
                    driveByJoystick(-forwardSpeed, turnSpeed);
//                    driveTurn(turnSpeed);
                    opMode.telemetry.addData("globalAng:", globalAng);
                    opMode.telemetry.addData("destination:", destination);
                    opMode.telemetry.addData("currentAng:", currentTurningAngle);
                    opMode.telemetry.update();
                    currentTurningAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            } else {
                while (currentTurningAngle < destination) {
                    driveByJoystick(-forwardSpeed, -turnSpeed);
//                    driveTurn(-turnSpeed);
                    opMode.telemetry.addData("globalAng:", globalAng);
                    opMode.telemetry.addData("destination:", destination);
                    opMode.telemetry.addData("currentAng:", currentTurningAngle);
                    opMode.telemetry.update();
                    currentTurningAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            }
        }

        globalAng = destination;
        stöp();
    }

    void turn180(double forwardSpeed, double turnSpeed, double destination) {
        double currentTurningAngle = globalAng;
        globalAng = destination;
        while (currentTurningAngle < 178 && currentTurningAngle > -178) {
            double correction = angleCorrection()/60;
            driveByJoystick(-forwardSpeed, correction);
            currentTurningAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
    }

    public void driveForward(double distance, double speed) {
        resetEncoder();
//        speed = speed / 2;

        while (getDistance() <= distance * 10) {
            double correction = angleCorrection() / 40;

            driveByJoystick(-speed, correction);
        }
        stöp();
    }
//
//    public void driveToWhite(double speed) {
//        while (colorSensor.getColor() != ColorSensor.ColorEnum.WHITE) {
//            double correction = angleCorrection() / 20;
//            driveByJoystick(-speed, correction);
//        }
//        stöp();
//    }
//
//    public void driveToBlue(double speed) {
//        while (colorSensor.getColor() != ColorSensor.ColorEnum.BLUE) {
//            double correction = angleCorrection() / 20;
//            driveByJoystick(-speed, correction);
//        }
//        stöp();
//    }
//
//    public void driveToRed(double speed) {
//        while (colorSensor.getColor() != ColorSensor.ColorEnum.RED) {
//            double correction = angleCorrection() / 20;
//            driveByJoystick(-speed, correction);
//        }
//        stöp();
//    }

    public void getAnglePerfect(int time){
        timer = new ElapsedTime();
        timer.reset();

        while(timer.milliseconds() < time){
            double correction = angleCorrection() / 60;
            driveByJoystick(0, correction);
        }
    }

    double angleCorrection() {
        double ang = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (ang > 90 && globalAng < -90)
            return ((-180 - globalAng) - (180 - ang));
        if (ang < -90 && globalAng > 90)
            return ((180 - globalAng) - (-180 - ang));
        return (ang - globalAng);
    }

    void resetEncoder() {
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    double getDistance() {
        double averagePosition = (rightMotor.getCurrentPosition() - leftMotor.getCurrentPosition()) / 2d;
        return Math.abs(averagePosition * 3 / COUNTS_PER_MM);
    }

    public void printXYZ() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        opMode.telemetry.addData("1", angles.firstAngle);
        opMode.telemetry.addData("2", angles.secondAngle);
        opMode.telemetry.addData("3", angles.thirdAngle);
        opMode.telemetry.update();
    }

}
