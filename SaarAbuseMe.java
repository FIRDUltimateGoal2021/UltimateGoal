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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.SaarDoMeBully;
import org.firstinspires.ftc.teamcode.UltimateGoal.Systems.ShooterSystem;

import java.util.function.Function;


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

@TeleOp(name = "Nevo's Absolutely Wonderful and Amazing I Want to Fucking Cum Code", group = "Linear Opmode")
@Disabled
public class SaarAbuseMe extends LinearOpMode {
    public enum StartingPos {
        REDL,
        REDR,
        BLUER,
        BLUEL
    }

    @Override
    public void runOpMode() {
        ShooterSystem shooterSystem = new ShooterSystem(this);

        NormalizedColorSensor pnimist;
        StartingPos jojo;
        boolean color = true; // true == blue
        boolean side = true; // true == left

        pnimist = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        SaarDoMeBully more = new SaarDoMeBully(pnimist);
        ElapsedTime timer = new ElapsedTime();

        Function<Double, Position> c = new Function<Double, Position>() {
            @Override
            public Position apply(Double aDouble) {
                Position f =  new Position();
                return f;
            }
        } ;
        while (!isStarted()) {
            if (gamepad1.x) {
                color = true;
            }
            if (gamepad1.b) {
                color = false;
            }

            if (gamepad1.dpad_left) {
                side = true;
            }
            if (gamepad1.dpad_right) {
                side = false;
            }
        }

        if (color) {
            if (side) {
                jojo = StartingPos.BLUEL;
            } else {
                jojo = StartingPos.BLUER;
            }
        } else {
            if (side) {
                jojo = StartingPos.REDL;
            } else {
                jojo = StartingPos.REDR;
            }
        }

        waitForStart();
        timer.reset();

        //driveAutonomouslybeter(0 , 5,c) ;

        // move to power shot
    }
}
