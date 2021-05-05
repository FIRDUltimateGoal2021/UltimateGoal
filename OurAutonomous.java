package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.CollectionSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.ColorSensor;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.ShootingSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.WobbleSystem;

@Autonomous(name="OurAutonomous", group="Linear Opmode")
public class OurAutonomous extends LinearOpMode {

    // TODO:
    // Detect number of rings
    // Drag Wobble to corresponding target
    // Park on white line


    DrivingSystem drivingSystem;
    ShootingSystem shootingSystem;
    CollectionSystem collectionSystem;
    WobbleSystem wobbleSystem;
    ColorSensor colorSensor;
    ElapsedTime timer = new ElapsedTime(100);

    @Override
    public void runOpMode() {
        drivingSystem = new DrivingSystem(this);
        shootingSystem = new ShootingSystem(this);
        collectionSystem = new CollectionSystem(this);
        wobbleSystem = new WobbleSystem(this);
        colorSensor = new ColorSensor(this);

        waitForStart();

        collectionSystem.on();
        shootingSystem.on();

        // Detect number of rings

        // Drag Wobble

        while (true) {
            drivingSystem.driveByJoystick(-1, 0);
            if (colorSensor.getColor() == ColorSensor.ColorEnum.WHITE) {
                drivingSystem.betterStöp();
                break;
            }
        }
    }
}
