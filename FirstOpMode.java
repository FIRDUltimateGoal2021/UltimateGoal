/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other matingials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INtingRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.CollectionSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.ShootingSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Utils.OurGamepad;
import org.firstinspires.ftc.teamcode.UltimateGoal.Utils.OurPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

/*
 * Use consistant conventions (decide between varName and var_name)
 * PLEASE fix the variable names in ANY part written by Nevo :).
 */

@TeleOp(name = "FirstOpMode", group = "Linear Opmode")
//@Disabled
public class FirstOpMode extends LinearOpMode {

    DrivingSystem drivingSystem;
    ShootingSystem shootingSystem;
    CollectionSystem collectionSystem;
    OurGamepad ourGamepad1;
    
    OpenCvInternalCamera phoneCam;
    OurPipeline pipeline;
    
    ElapsedTime timer = new ElapsedTime(100);

    @Override
    public void runOpMode() {
        drivingSystem = new DrivingSystem(this);
        shootingSystem = new ShootingSystem(this);
        collectionSystem = new CollectionSystem(this);
        ourGamepad1 = new OurGamepad(gamepad1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new OurPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        final double loadingTime = 1;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Joysticks
            drivingSystem.driveByJoystick(-gamepad2.left_stick_y, gamepad2.right_stick_x);
            shootingSystem.changeAngle(
                    shootingSystem.currentAngle
                            + 0.5 * gamepad1.right_stick_y
            );

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);

            // Right Toggle: Toggle Collection System
            if (ourGamepad1.buttonPress("Rt")) {
                collectionSystem.toggle();
            }

            // Left Toggle: Toggle Shooting System motor
            if (ourGamepad1.buttonPress("Lt")) {
                shootingSystem.toggle();
            }

            // Right Bumper: Toggle Collection System spit
            if (ourGamepad1.buttonPress("Rb")) {
                collectionSystem.spit();
            }

            // Right Bumper: Toggle Shooting System spit
            if (ourGamepad1.buttonPress("Lb")) {
                shootingSystem.spit();
            }

            // Button a: Toggle between loading position and shoot (servo)
            if (ourGamepad1.buttonPress("a")) {
                shootingSystem.shootLoad();
            }

            ourGamepad1.update();
            telemetry.update();
        }
    }
}