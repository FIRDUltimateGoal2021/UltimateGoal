package org.firstinspires.ftc.teamcode.UltimateGoal.Utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.ShootingSystem;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Routs {

    LinearOpMode opMode;
    DrivingSystem drivingSystem;
    ShootingSystem shootingSystem;
    ElapsedTime timer;

    int color = 0;

    public Routs(String color, LinearOpMode opMode) {
        if (color.equals("blue"))
            this.color = 1;
        else if (color.equals("red"))
            this.color = -1;

        this.opMode = opMode;
        drivingSystem = new DrivingSystem(opMode);
        shootingSystem = new ShootingSystem(opMode);
        timer = new ElapsedTime();
    }

    public void rightC() {
        drivingSystem.driveForward(280, 0.5);
        drivingSystem.turn(90 * color, 0.5);
        drivingSystem.driveForward(75, 0.5);
        drivingSystem.driveForward(75, -0.5);
        drivingSystem.turn(-90 * color, 0);
        drivingSystem.driveForward(100, -0.5);
//        shootingSystem.toggle();
//        drivingSystem.turn(180 * color, 0);
//        shootingSystem.shootLoad();
//        shootingSystem.toggle();
//        drivingSystem.driveForward(10, 0.5);
//        shootingSystem.shootLoad();
        opMode.requestOpModeStop();
    }

    public void rightB() {
        drivingSystem.driveForward(220, 0.5);
        drivingSystem.turn(90, 0.5);
        drivingSystem.turn(-90, -0.5);
//        shootingSystem.toggle();
//        drivingSystem.turn(180, 0);
//        shootingSystem.shootLoad();
//        drivingSystem.driveForward(30, -0.5);
//        shootingSystem.shootLoad();
//        shootingSystem.toggle();
        opMode.requestOpModeStop();
    }

    public void rightA() {
        drivingSystem.driveForward(160, 0.5);
        drivingSystem.turn(90, 0.5);
        drivingSystem.driveForward(75, 0.5);
        drivingSystem.driveForward(75,-0.5);
//        drivingSystem.turn(90, 0);
//        shootingSystem.toggle();
//        shootingSystem.load();
//        drivingSystem.driveForward(50, 0.5);
//        shootingSystem.shoot();
//        sleep(500);
//        shootingSystem.toggle();
//        shootingSystem.load();
//        drivingSystem.driveForward(50, -0.5);
//        sleep(2000);
        opMode.requestOpModeStop();
    }

    void leftC(){

    }




    void sleep(int time){
        timer.reset();
        while(timer.milliseconds() < time){
            // ahhhhhhhhhhhhhhhhhhhh
        }
    }
}
