package org.firstinspires.ftc.teamcode.UltimateGoal.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
	private ElapsedTime runtime = new ElapsedTime();
	private double kp, ki, kd;
	double lastError = Double.NaN;
	double proportional, integral = 0, derivitive = 0;
	public PIDController(double p,double i,double d) {
		kp = p;
		ki = i;
		kd = d;
		runtime.reset();
	}
	public double getValue(double Error){
		double deltaTime = runtime.milliseconds();
		proportional = kp*Error;
		if(lastError != Double.NaN){
			integral += ki*(Error-lastError)*(deltaTime);
			derivitive = kd*((Error-lastError)/(deltaTime));
		}
		lastError=Error;
		runtime.reset();
		double value = proportional + integral + derivitive;
	    if(value < -1){
	    	return -1;
	    }
	    if(value > 1){
	    	return 1;
	    }

	    return value;
	}

	public double getProportional() {
		return proportional/kp;
	}

	public double getIntegral() {
		return integral/ki;
	}

	public double getDerivitive() {
		return derivitive/kd;
	}

	public double getKp() {
		return kp;
	}

	public void setKp(double kp) {
		this.kp = kp;
	}

	public double getKi() {
		return ki;
	}

	public void setKi(double ki) {
		integral = integral*ki/this.ki;
		this.ki = ki;
	}

	public double getKd() {
		return kd;
	}

	public void setKd(double kd) {
		this.kd = kd;
	}
}
