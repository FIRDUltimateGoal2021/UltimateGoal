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
        drivingSystem.driveForward(300, 0.5);
        drivingSystem.turn(90 * color, 0.5);
        drivingSystem.driveForward(75, 0.5);
        drivingSystem.driveForward(75, -0.5);
        drivingSystem.turn(-90 * color, 0);
        drivingSystem.driveForward(125, -0.5);
        shootingSystem.toggle();
        drivingSystem.turn(180 * color, 0);
        shootingSystem.shootLoad();
        shootingSystem.toggle();
        drivingSystem.driveForward(10, 0.5);
        shootingSystem.shootLoad();
        opMode.requestOpModeStop();
    }

    public void rightB() {
        drivingSystem.driveForward(220, 0.5);
        drivingSystem.turn(90 * color, 0.5);
        drivingSystem.turn(-90 * color, -0.5);
        shootingSystem.toggle();
        drivingSystem.turn(180 * color, 0);
        shootingSystem.shootLoad();
        drivingSystem.driveForward(30, -0.5);
        shootingSystem.shootLoad();
        shootingSystem.toggle();
        opMode.requestOpModeStop();
    }

    public void rightA() {
        drivingSystem.driveForward(160, 0.5);
        drivingSystem.turn(90 * color, 0.5);
        drivingSystem.driveForward(110, 0.5);
        drivingSystem.driveForward(110,-0.5);
        drivingSystem.turn(90 * color, 0);
        shootingSystem.toggle();
        shootingSystem.load();
        drivingSystem.driveForward(100, 0.5);
        drivingSystem.turn(5 * color,0);
        shootingSystem.shoot();
        sleep(500);
        shootingSystem.toggle();
        shootingSystem.load();
        drivingSystem.driveForward(50, -0.5);
        opMode.requestOpModeStop();
    }

    public void leftC(){
        drivingSystem.turn(45 * color,0.7);
        drivingSystem.turn(-45 * color,0.7);
        drivingSystem.driveForward(300,0.5);
        drivingSystem.driveForward(140,-0.5);
        drivingSystem.turn(180 * color,0);
        shootingSystem.on();
        shootingSystem.load();
        drivingSystem.turn(-30 * color,0);
        shootingSystem.shoot();
        sleep(500);
        drivingSystem.driveForward(55,-0.5);
        shootingSystem.off();
        shootingSystem.load();
        opMode.requestOpModeStop();
    }

    public void leftB(){
        drivingSystem.driveForward(240,0.5);
        drivingSystem.turn(-90 * color,0.5);
        drivingSystem.turn(90 * color,-0.5);
        drivingSystem.driveForward(20,-0.5);
        opMode.requestOpModeStop();
    }

    public void leftA(){
        drivingSystem.turn(45 * color,0.7);
        drivingSystem.turn(-45 * color,0.7);
        drivingSystem.driveForward(170,0.5);
        drivingSystem.turn(90,-0.5);
        shootingSystem.on();
        shootingSystem.load();
        drivingSystem.turn(80,0);
        drivingSystem.driveForward(80,0.5);
        shootingSystem.shoot();
        sleep(500);
        drivingSystem.driveForward(25,-0.5);
        shootingSystem.off();
        shootingSystem.load();
        opMode.requestOpModeStop();
    }




    void sleep(int time){
        timer.reset();
        while(timer.milliseconds() < time){
            // ahhhhhhhhhhhhhhhhhhhh
        }
    }
}
