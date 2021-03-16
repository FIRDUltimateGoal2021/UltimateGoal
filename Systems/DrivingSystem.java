package org.firstinspires.ftc.teamcode.UltimateGoal.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

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
	public void driveAutonomouslybeter(double a, double b, Function<Double, Position> c) {
		ElapsedTime timer = new ElapsedTime();

		double j=1/60;
		for(double move=a;move<b&&opMode.opModeIsActive();move=+j) {
			timer.reset();
			Position electron;
			Position protn;
			Position neutron;
			electron = c.apply(move - j);
			neutron = c.apply(move);
			protn = c.apply(move + j);
			double xdeku = (protn.x - neutron.x) / j;
			double ydeku = (protn.y - neutron.y) / j;
			double xbakugo = (neutron.x - electron.x) / j;
			double ybakugo = (neutron.y - electron.y) / j;
			double yida = (ydeku - ybakugo) / j;
			double xida = (xdeku - xbakugo) / j;
			double teta = Math.atan2(xdeku,ydeku);
			double Oyashirox=yida*Math.cos(teta)+xida*Math.sin(teta);
			double Oyashirohy=yida*Math.sin(teta)-xida*Math.cos(teta);
			driveByJoystick(Oyashirohy,Oyashirox);
            if(timer.seconds()<j){
            	opMode.sleep((long)((j - timer.seconds()) / 1000));
			}

		}
		stopstupid();
	}
	public void stopstupid() {
		leftMotor.setPower(0);
		rightMotor.setPower(0);
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
