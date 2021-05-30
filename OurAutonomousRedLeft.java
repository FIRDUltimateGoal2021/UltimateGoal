package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.CollectionSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.ShootingSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Utils.OurPipeline;
import org.firstinspires.ftc.teamcode.UltimateGoal.Utils.Routs;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "OurAutonomousRL", group = "Linear Opmode")
public class OurAutonomousRedLeft extends LinearOpMode {

    // TODO:
    // Detect number of rings
    // Drag Wobble to corresponding target
    // Park on white line


    DrivingSystem    drivingSystem;
    ShootingSystem   shootingSystem;
    CollectionSystem collectionSystem;

    OpenCvInternalCamera phoneCam;
    OurPipeline          pipeline;

    ElapsedTime timer = new ElapsedTime(100);

    Routs routs;

    @Override
    public void runOpMode() {
        drivingSystem    = new DrivingSystem(this);
        shootingSystem   = new ShootingSystem(this);
        collectionSystem = new CollectionSystem(this);
        routs = new Routs("red", this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new OurPipeline(243, 143);
        phoneCam.setPipeline(pipeline);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        timer.reset();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);

            if (timer.seconds() >= 2) {
                switch (pipeline.position) {
                    case NONE:
                        routs.rightA();
                        return;
                    case ONE:
                        routs.rightB();
                        return;
                    case FOUR:
                        routs.rightC();
                }
            }

            telemetry.update();
        }
    }
}
