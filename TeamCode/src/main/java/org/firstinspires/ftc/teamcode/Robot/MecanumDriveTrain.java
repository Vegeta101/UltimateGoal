package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.logitech.LogitechGamepadF310;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.R;

import java.util.List;

public class MecanumDriveTrain {
    DcMotorEx       left_front;
    DcMotorEx       left_back;
    DcMotorEx       right_front;
    DcMotorEx       right_back;

    Robot           robot;

    public MecanumDriveTrain(Robot robot) {
        this.robot        = robot;
    }

    public void init() {
        left_front  = robot.opMode.hardwareMap.get(DcMotorEx.class, "left_front");
        right_front = robot.opMode.hardwareMap.get(DcMotorEx.class, "right_front");
        left_back = robot.opMode.hardwareMap.get(DcMotorEx.class, "left_back");
        right_back =robot.opMode.hardwareMap.get(DcMotorEx.class, "right_back");
        left_front.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        right_front.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        left_back.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        right_back.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void DriverControlled_Drive(){
        double y = robot.opMode.gamepad1.left_stick_y;  // Control to Drive Forward / Back
        double x = robot.opMode.gamepad1.left_stick_x;  // Strafe Control
        double rot = robot.opMode.gamepad1.right_stick_x; // Rotation Control
        y = y * -1;  // Pressing up on Controller is -1 in Y axis so multiply by -1 to make driving more intuitive.

        // Init and Set all Power Variables to 0
        double power_left_front = 0;
        double power_left_back = 0;
        double power_right_front = 0;
        double power_right_back = 0;
        // End Power Init

        // Calculate Each Wheel Drive Power based on X, Y and Rot values from Control Pad.
        power_left_front = y + x + rot;
        power_left_back = y - x + rot;
        power_right_front = y - x - rot;
        power_right_back = y + x - rot;

        // Set Power on Each Wheel Motor.
        left_front.setPower(power_left_front);
        left_back.setPower(power_left_back);
        right_front.setPower(power_right_front);
        right_back.setPower(power_right_back);

        // Telemetry to Phone for Actual Power to Wheel Motors. Telemetry updated in Op Mode.
        robot.opMode.telemetry.addLine("Front Wheel Power ")
                .addData("Left","%.3f", power_left_front)
                .addData("Right", "%.3f", power_right_front);
        robot.opMode.telemetry.addLine(" Rear Wheel Power ")
                .addData("Left", "%.3f", power_left_back)
                .addData("Right","%.3f",  power_right_back);
    }
}
