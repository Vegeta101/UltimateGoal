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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous_Testing", group="Production")
public class Autonomous_Testing extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_Map robot = new Hardware_Map();
    Webcam_Object_Detection webcam = new Webcam_Object_Detection();
    private ElapsedTime runtime = new ElapsedTime();

    static final int COUNTS_PER_MOTOR_REV = 28;    // Motor with 1:1 gear ratio
    static final double DRIVE_GEAR_REDUCTION = 10.5;     // Rev Ultraplanetary Motor 12:1 but actual is 10.5:1
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
    static final double COUNTS_PER_INCH_DOUBLE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        String ringdetect = "";
        ringdetect = webcam.detect();
        telemetry.addData("ring patter", ringdetect);
        telemetry.addData("", "----------------------------");

        telemetry.addData(">", "Press Play to Start");
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Status", "Init Hardware");
        //telemetry.addData("Webcam View", webcam.webcamview);
       // telemetry.update();

        // Send telemetry message to indicate successful Encoder reset
//

        // Activates and detects the amount of disks present on the field


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        if (opModeIsActive()) {

            //Moving towards the foundation
            mecanumDrive(10, 10, 10, 10, .17, 2500);

            //while (robot.left_front.isBusy() || robot.left_back.isBusy() || robot.right_back.isBusy() || robot.right_front.isBusy()) {
            telemetry.addLine("left wheels | ")
                    .addData("front", robot.left_front.getCurrentPosition())
                    .addData("back", robot.left_back.getCurrentPosition());
            telemetry.addLine("right wheels | ")
                    .addData("front", robot.right_front.getCurrentPosition())
                    .addData("back", robot.right_back.getCurrentPosition());

            telemetry.update();
            robot.left_front.setPower(0);
            robot.right_front.setPower(0);
            robot.right_back.setPower(0);
            robot.left_back.setPower(0);
        }

    }

    public void mecanumDrive(double right_front, double right_back,
                             double left_front, double left_back,
                             double motor_power, long wait) {
        int new_left_front;
        int new_left_back;
        int new_right_front;
        int new_right_back;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            new_left_front = robot.left_front.getCurrentPosition() + (int) (COUNTS_PER_INCH_DOUBLE * left_front);
            new_left_back = robot.left_back.getCurrentPosition() + (int) (COUNTS_PER_INCH_DOUBLE * left_back);
            new_right_front = robot.right_front.getCurrentPosition() + (int) (COUNTS_PER_INCH_DOUBLE * right_front);
            new_right_back = robot.right_back.getCurrentPosition() + (int) (COUNTS_PER_INCH_DOUBLE * right_back);

            robot.left_front.setTargetPosition(new_left_front);
            robot.left_back.setTargetPosition(new_left_back);
            robot.right_front.setTargetPosition(new_right_front);
            robot.right_back.setTargetPosition(new_right_back);

            robot.left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.left_front.setPower(Math.abs(motor_power));
            robot.left_back.setPower(Math.abs(motor_power));
            robot.right_front.setPower(Math.abs(motor_power));
            robot.right_back.setPower(Math.abs(motor_power));

            sleep(wait);


        }

    }
}


/*
 *  Method to perfmorm a relative move, based on encoder counts.
 *  Encoders are not reset as the move is based on the current position.
 *  Move will stop if any of three conditions occur:
 *  1) Move gets to the desired position
 *  2) Move runs out of time
 *  3) Driver stops the opmode running.
 */