package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.Function;

public class DrivingSystem {

	LinearOpMode opMode;
	DcMotor leftMotor;
	DcMotor rightMotor;

	public DrivingSystem(LinearOpMode opMode){
		this.opMode = opMode;
		leftMotor = opMode.hardwareMap.get(DcMotor.class,"left_drive");
		rightMotor = opMode.hardwareMap.get(DcMotor.class,"right_drive");
	}

	public void driveByJoystick(double horizontal, double vertical){
		double left = vertical-horizontal;
		double right = vertical+horizontal;
		if(Math.abs(left)>1 || Math.abs(right)>1){
			left = left * Math.max(Math.abs(left),Math.abs(right));
			right = right * Math.max(Math.abs(left),Math.abs(right));
		}
		leftMotor.setPower(left);
		rightMotor.setPower(right);
	}

	public void driveAutonomously(double[][] way) {
		 for (double[] segment : way) {
		 	rotate(segment[0]);
		 	driveForward(segment[1]);
		 }
	}
	public void driveAutonomouslybeter(int a, int b, Function<double,>) {
		for (double[] segment : way) {
			rotate(segment[0]);
			driveForward(segment[1]);
		}
	}

	public void driveForward(double distance) {
	}

	/**
	 *
	 * @param angle in degrees
	 */
	public void rotate(double angle) {
	}
}
