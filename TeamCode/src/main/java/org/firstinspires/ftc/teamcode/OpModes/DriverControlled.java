
package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware_Map;
import org.firstinspires.ftc.teamcode.Robot.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Robot.*;

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

@TeleOp(name="DriverControlled")
public class DriverControlled extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot  = new Robot(this);

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.getDriveTrain().init();


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Initialized");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double speed_reduction    = 1;

        //PROGRAM STARTS HERE -----------------------------------------------------------------------------------------------

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            double power_left_front = 0;
            double power_left_back = 0;
            double power_right_front = 0;
            double power_right_back = 0;

            power_left_front = y + x + rot;
            power_left_back = y - x + rot;
            power_right_front = y - x - rot;
            power_right_back = y + x - rot;
            
            robot.getDriveTrain().DriverControlled_Drive(power_left_front, power_left_back, power_right_front, power_right_back);

            telemetry.addData("Left Front", power_left_front);
            telemetry.addData("Right Front", power_right_front);
            telemetry.addData("Left Rear", power_left_back);
            telemetry.addData("Right Rear", power_right_back);
            telemetry.update();
            // PROGRAM ENDS HERE -------------------------------------------------------------------------------------
        }
    }
}

