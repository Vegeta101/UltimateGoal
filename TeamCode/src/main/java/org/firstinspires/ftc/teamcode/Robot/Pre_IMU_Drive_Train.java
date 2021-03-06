package org.firstinspires.ftc.teamcode.Robot;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------


public class Pre_IMU_Drive_Train {
    static final int COUNTS_PER_MOTOR_REV = 28;    // Motor with 1:1 gear ratio
    static final double DRIVE_GEAR_REDUCTION = 10.5;     // Rev Ultraplanetary Motor 12:1 but actual is 10.5:1
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
    static final double COUNTS_PER_INCH_DOUBLE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    Orientation     angles;
    DcMotorEx       left_front;
    DcMotorEx       left_back;
    DcMotorEx       right_front;
    DcMotorEx       right_back;
    DcMotorEx       arm_motor;
    Servo           arm_servo;
    ColorSensor     colorSensor;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final int SCALE_FACTOR = 255;

    Robot           robot;

    public Pre_IMU_Drive_Train(Robot robot) {
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
        colorSensor = (ColorSensor) robot.opMode.hardwareMap.get(HardwareDevice.class, "sensor_color");
        // Set all motors to zero power
        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    public void AutonomousDrive(double leftFront, double leftBack,
                                double rightFront, double rightBack,
                                double motor_power){
        int new_left_front;
        int new_left_back;
        int new_right_front;
        int new_right_back;

        // Determine new target position, and pass to motor controller
        new_left_front = left_front.getCurrentPosition() + (int) (COUNTS_PER_INCH_DOUBLE * leftFront);
        new_left_back = left_back.getCurrentPosition() + (int) (COUNTS_PER_INCH_DOUBLE * leftBack);
        new_right_front = right_front.getCurrentPosition() + (int) (COUNTS_PER_INCH_DOUBLE * rightFront);
        new_right_back = right_back.getCurrentPosition() + (int) (COUNTS_PER_INCH_DOUBLE * rightBack);
        right_back.setTargetPosition(new_left_front);
        left_back.setTargetPosition(new_left_back);
        right_front.setTargetPosition(new_right_front);
        left_front.setTargetPosition(new_right_back);

        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left_back.setPower(Math.abs(motor_power));
        right_back.setPower(Math.abs(motor_power));
        left_front.setPower(Math.abs(motor_power));
        right_front.setPower(Math.abs(motor_power));


        while (right_back.isBusy() && left_back.isBusy() && left_front.isBusy() && right_front.isBusy()){

        }

    }
    public void moveToColor(String targetColor, double motor_power){
        Color.RGBToHSV(colorSensor.red() * SCALE_FACTOR, colorSensor.green() * SCALE_FACTOR, colorSensor.blue() * SCALE_FACTOR, hsvValues);

        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Color.RGBToHSV(colorSensor.red() * SCALE_FACTOR, colorSensor.green() * SCALE_FACTOR, colorSensor.blue() * SCALE_FACTOR, hsvValues);
        right_back.setPower(motor_power);
        left_back.setPower(motor_power);
        right_front.setPower(motor_power);
        left_front.setPower(motor_power);



        if (targetColor == "blue"){
            while (colorSensor.blue() < 250 && (colorSensor.red() > 150) && (colorSensor.alpha() < 600)) {
                Color.RGBToHSV( colorSensor.red() * SCALE_FACTOR, colorSensor.green() * SCALE_FACTOR, colorSensor.blue() * SCALE_FACTOR, hsvValues);
                robot.opMode.telemetry.addData("Red", colorSensor.red());
                robot.opMode.telemetry.addData("Blue", colorSensor.blue());
                robot.opMode.telemetry.addData("Green", colorSensor.green());
                robot.opMode.telemetry.addData("Clear", colorSensor.alpha());
                robot.opMode.telemetry.update();
            }
        }
        else if (targetColor == "red"){
            while (colorSensor.red() < 400 && colorSensor.alpha() < 600) {
                Color.RGBToHSV( colorSensor.red() * SCALE_FACTOR, colorSensor.green() * SCALE_FACTOR, colorSensor.blue() * SCALE_FACTOR, hsvValues);
                robot.opMode.telemetry.addData("Red", colorSensor.red());
                robot.opMode.telemetry.addData("Blue", colorSensor.blue());
                robot.opMode.telemetry.addData("Green", colorSensor.green());
                robot.opMode.telemetry.addData("Clear", colorSensor.alpha());
                robot.opMode.telemetry.update();
            }
        }
        else if (targetColor == "white"){
            while (colorSensor.red() < 700 && colorSensor.blue() < 600 && colorSensor.green() < 750 && colorSensor.alpha() < 2000) {
                Color.RGBToHSV( colorSensor.red() * SCALE_FACTOR, colorSensor.green() * SCALE_FACTOR, colorSensor.blue() * SCALE_FACTOR, hsvValues);
                robot.opMode.telemetry.addData("Red", colorSensor.red());
                robot.opMode.telemetry.addData("Blue", colorSensor.blue());
                robot.opMode.telemetry.addData("Green", colorSensor.green());
                robot.opMode.telemetry.addData("Clear", colorSensor.alpha());
                robot.opMode.telemetry.update();
            }
        }


        //try to figure out how to motor brake
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
    }
    public void wobbleGrip(int motor_position, double motor_power, double servo) {
        arm_motor.setTargetPosition(motor_position);
        arm_motor.setPower(motor_power);
        arm_servo.setPosition(servo);
    }
    public void IMU_drive() {
        angles   = robot.imu_hub1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}
