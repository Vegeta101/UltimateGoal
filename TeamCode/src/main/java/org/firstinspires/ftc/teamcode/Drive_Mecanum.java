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

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Driver Controlled")
public class Drive_Mecanum extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_Map   robot  = new Hardware_Map();

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Initialized");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double speed_reduction    = 1;

        //PROGRAM STARTS HERE -----------------------------------------------------------------------------------------------

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Fancy math to drive mecanum wheels
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotangle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            final double V1 = r * Math.cos(robotangle) + rightX;
            final double V2 = r * Math.sin(robotangle) - rightX;
            final double V3 = r * Math.sin(robotangle) + rightX;
            final double V4 = r * Math.cos(robotangle) - rightX;

            /*
            if (gamepad1.a = true) {
                speed_reduction = .25;
            }

            else if (gamepad1.b = true) {
                speed_reduction = .5;
            }

            else if (gamepad1.x = true) {
                speed_reduction = .75;
            }
            else if (gamepad1.y = true) {
                speed_reduction = 1;
            }
            
             */
            robot.left_front.setPower(V1 * speed_reduction);
            robot.right_front.setPower(V2 * speed_reduction);
            robot.left_back.setPower(V3 * speed_reduction);
            robot.right_back.setPower(V4 * speed_reduction);

            telemetry.addData("V1", V1);
            telemetry.addData("V2", V2);
            telemetry.addData("V3", V3);
            telemetry.addData("V4", V4);
            telemetry.update();
            // PROGRAM ENDS HERE -------------------------------------------------------------------------------------
        }
    }
}
