package org.firstinspires.ftc.teamcode.UltimateGoal.Utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.CollectionSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.ShootingSystem;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Routs {

    LinearOpMode opMode;
    DrivingSystem drivingSystem;
    ShootingSystem shootingSystem;
    CollectionSystem collectionSystem;
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
        collectionSystem = new CollectionSystem(opMode);
        timer = new ElapsedTime();
    }

    public void rightC() {
        drivingSystem.driveForward(300, 0.5);
        drivingSystem.turn(90 * color, 0.5);
        drivingSystem.driveForward(75, 0.5);
        drivingSystem.driveForward(40, -0.5);
        drivingSystem.turn(-90 * color, 0);
        drivingSystem.driveForward(155, -0.5);
        shootingSystem.on();
        drivingSystem.turn(160 * color, 0);
        sleep(500);
        shootingSystem.shoot();
        sleep(500);
        shootingSystem.load();
        shootingSystem.off();
        drivingSystem.driveForward(20, -0.5);
        opMode.requestOpModeStop();
    }

    public void rightB() {
        drivingSystem.driveForward(240, 0.5);
        drivingSystem.turn(90 * color, 0.5);
        drivingSystem.driveForward(15,-0.5);
        drivingSystem.turn(90 * color, -0.5);
        drivingSystem.driveForward(180, 0.5);
        shootingSystem.on();
        shootingSystem.load();
        drivingSystem.turn(2.5 * color, 0);
        sleep(500);
        shootingSystem.shoot();
        sleep(500);
        drivingSystem.driveForward(21, -0.5);
        shootingSystem.load();
        shootingSystem.off();
        opMode.requestOpModeStop();
    }

    public void rightA() {
        drivingSystem.driveForward(160, 0.5);
        drivingSystem.turn(90 * color, 0.5);
        sleep(1000);
        drivingSystem.driveForward(100, 0.5);
        drivingSystem.driveForward(60, -0.5);
        drivingSystem.turn(90 * color, 0);
        shootingSystem.on();
        shootingSystem.load();
        drivingSystem.driveForward(60, 0.5);
        drivingSystem.turn(1.5*color,0);
        sleep(500);
        shootingSystem.shoot();
        sleep(500);
        shootingSystem.off();
        shootingSystem.load();
        drivingSystem.driveForward(20, -0.5);
        opMode.requestOpModeStop();
    }

    public void leftC() {
        drivingSystem.turn(25 * color, 0.7);
        drivingSystem.turn(-25 * color, 0.7);
        drivingSystem.driveForward(270, 0.5);
        drivingSystem.driveForward(160, -0.5);
        shootingSystem.load();
        shootingSystem.on();
        drivingSystem.turn(170 * color, 0);
        drivingSystem.getAnglePerfect(2000);
        shootingSystem.shoot();
        sleep(500);
        drivingSystem.driveForward(10, -0.5);
        shootingSystem.off();
        shootingSystem.load();
        opMode.requestOpModeStop();
    }

    public void leftB() {
        drivingSystem.driveForward(240, 0.5);
        drivingSystem.turn(-90 * color, 0.5);
        drivingSystem.driveForward(20, 0.5);
        drivingSystem.driveForward(20, -0.5);
        drivingSystem.turn(-90 * color, -0.25);
        drivingSystem.driveForward(145, 0.5);
        shootingSystem.on();
        shootingSystem.load();
        drivingSystem.turn(-,0);
        drivingSystem.getAnglePerfect(2000);
        shootingSystem.shoot();
        sleep(500);
        drivingSystem.driveForward(20, -0.5);
        shootingSystem.load();
        opMode.requestOpModeStop();
    }

    public void leftA() {
        drivingSystem.turn(25 * color, 0.7);
        drivingSystem.turn(-25 * color, 0.7);
        drivingSystem.driveForward(130, 0.5);
        drivingSystem.turn(90 * color, -0.5);
        drivingSystem.turn(90 * color, 0);
        drivingSystem.driveForward(38, 0.5);
        drivingSystem.getAnglePerfect(2000);
        resetForNewShot(false);
        shootingSystem.shoot();
        sleep(500);
        resetForNewShot(true);
        shootingSystem.shoot();
        sleep(500);
        drivingSystem.driveForward(15, -0.5);
        shootingSystem.off();
        shootingSystem.load();
        opMode.requestOpModeStop();
    }

    void resetForNewShot(boolean withCollection){
        shootingSystem.off();
        shootingSystem.load();
        if(withCollection)
        collectionSystem.on();
        sleep(3000);
        collectionSystem.off();
        shootingSystem.load();
        sleep(500);
        shootingSystem.shoot();
        sleep(500);
        shootingSystem.load();
        sleep(500);
        shootingSystem.shoot();
        sleep(500);
        shootingSystem.load();
        shootingSystem.on();
        sleep(1000);
    }

    void sleep(int time) {
        timer.reset();
        while (timer.milliseconds() < time) {
            // ahhhhhhhhhhhhhhhhhhhh
        }
    }
}
