package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.CollectionSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.ColorSensor;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.ShootingSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.WobbleSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Utils.OurPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "OurAutonomous", group = "Linear Opmode")
public class OurAutonomousBlueRight extends LinearOpMode {

    // TODO:
    // Detect number of rings
    // Drag Wobble to corresponding target
    // Park on white line


    DrivingSystem    drivingSystem;
    ShootingSystem   shootingSystem;
    CollectionSystem collectionSystem;
    WobbleSystem     wobbleSystem;
    ColorSensor      colorSensor;

    OpenCvInternalCamera phoneCam;
    OurPipeline          pipeline;

    ElapsedTime timer = new ElapsedTime(100);

    @Override
    public void runOpMode() {
        drivingSystem    = new DrivingSystem(this);
        shootingSystem   = new ShootingSystem(this);
        collectionSystem = new CollectionSystem(this);
        wobbleSystem     = new WobbleSystem(this);
        colorSensor      = new ColorSensor(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new OurPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        collectionSystem.on();
        shootingSystem.on();

        // Detect number of rings

        // Drag Wobble

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);

            switch (pipeline.position) {
                case NONE:
                    A();
                    return;
                case ONE:
                    B();
                    return;
                case FOUR:
                    C();
            }
        }
    }

    public void A() {
        drivingSystem.driveToWhite(1);
        drivingSystem.turn(-90);
        drivingSystem.driveToBlue(1);
    }

    public void B() {
        drivingSystem.driveToWhite(-1);
        drivingSystem.turn(-45);
        drivingSystem.driveForward(1, -0.5);
        drivingSystem.driveToWhite(0.5);
    }

    public void C() {
        drivingSystem.driveToBlue(-1);
        drivingSystem.driveToBlue(-1);
        drivingSystem.turn(-80);
        drivingSystem.driveForward(1, -0.5);
        drivingSystem.driveForward(1, 0.5);
        drivingSystem.turn(80);
        drivingSystem.driveToWhite(1);
    }
}
