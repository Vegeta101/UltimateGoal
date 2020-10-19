package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

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
    public void DriverControlled_Drive(double leftfront, double leftrear, double rightfront, double rightrear){
        left_front.setPower(leftfront);
        left_back.setPower(leftrear);
        right_front.setPower(rightfront);
        right_back.setPower(rightrear);

    }
}
