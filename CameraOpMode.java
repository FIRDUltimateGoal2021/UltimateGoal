package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.UltimateGoal.Utils.OurPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "CameraOpMode", group = "Linear Opmode")
public class CameraOpMode extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    OurPipeline          pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new OurPipeline(50, 150);
        phoneCam.setPipeline(pipeline);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
        }
    }
}
