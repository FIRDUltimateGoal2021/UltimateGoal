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
 * other materials provided with the distribution.
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
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.CollectionSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.ShooterSystem;


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

@TeleOp(name = "FirstOpMode", group = "Linear Opmode")
//@Disabled
public class FirstOpMode extends LinearOpMode {

    DrivingSystem drivingSystem;
    ShooterSystem shooterSystem;
    CollectionSystem collectionSystem;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        drivingSystem = new DrivingSystem(this);
        shooterSystem = new ShooterSystem(this);
        collectionSystem = new CollectionSystem(this);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        int b = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            shooterSystem.motor.setPower(1);
            drivingSystem.driveByJoystick(gamepad2.left_stick_x, -gamepad2.left_stick_y);
            shooterSystem.changeAngle(
                    shooterSystem.currentHorizontalAngle
                            - 0.1 * gamepad1.right_stick_y
            );

            if (gamepad1.a) {
                shooterSystem.shoot();
                timer.reset();
                if (timer.seconds() >= 1) {
                    shooterSystem.load();
                }
            }

            if (gamepad1.b) {
                if (b == 0) {
                    collectionSystem.toggle();
                }
                b += 1;
            } else if (b > 0) {
                b = 0;
            }

            if (gamepad1.x) {
                System.out.println("x");
            }

            if (gamepad1.y) {
                System.out.println("y");
            }
        }
    }

}
