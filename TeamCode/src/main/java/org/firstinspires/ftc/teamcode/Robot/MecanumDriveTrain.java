package org.firstinspires.ftc.teamcode.Robot;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.logitech.LogitechGamepadF310;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//------------------------------------------------------------------------------
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//------------------------------------------------------------------------------

import org.firstinspires.ftc.teamcode.R;

import java.util.List;

public class MecanumDriveTrain {
    static final int COUNTS_PER_MOTOR_REV           = 28;      // Motor with 1:1 gear ratio
    static final double DRIVE_GEAR_REDUCTION        = 10.5;    // Rev Ultraplanetary Motor 12:1 but actual is 10.5:1
    static final double WHEEL_DIAMETER_INCHES       = 3.0;     // For figuring circumference
    static final double COUNTS_PER_INCH_DOUBLE      = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double HEADING_THRESHOLD   = 1;
    static final double     P_TURN_COEFF            = 0.1;
    static final double     P_DRIVE_COEFF           = 0.15;
    
    DcMotorEx       left_front;
    DcMotorEx       left_back;
    DcMotorEx       right_front;
    DcMotorEx       right_back;
    DcMotorEx       arm_motor;
    Servo           arm_servo;
    ColorSensor     colorSensor;

    float hsvValues[]      = {0F, 0F, 0F};
    final float values[]   = hsvValues;
    final int SCALE_FACTOR = 255;

    Robot robot;

    public MecanumDriveTrain(Robot robot) {
        this.robot = robot;
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
        double y   = robot.opMode.gamepad1.left_stick_y;  // Control to Drive Forward / Back
        double x   = robot.opMode.gamepad1.left_stick_x;  // Strafe Control
        double rot = robot.opMode.gamepad1.right_stick_x; // Rotation Control
        y = y * -1;  // Pressing up on Controller is -1 in Y axis so multiply by -1 to make driving more intuitive.

        // Init and Set all Power Variables to 0
        double power_left_front  = 0;
        double power_left_back   = 0;
        double power_right_front = 0;
        double power_right_back  = 0;
        // End Power Init

        // Calculate Each Wheel Drive Power based on X, Y and Rot values from Control Pad.
        power_left_front  = y + x + rot;
        power_left_back   = y - x + rot;
        power_right_front = y - x - rot;
        power_right_back  = y + x - rot;

        // Set Power on Each Wheel Motor.
        left_front.setPower(power_left_front);
        left_back.setPower(power_left_back);
        right_front.setPower(power_right_front);
        right_back.setPower(power_right_back);

        // robot.opMode.telemetry to Phone for Actual Power to Wheel Motors. robot.opMode.telemetry updated in Op Mode.
        robot.opMode.telemetry.addLine("Front Wheel Power ")
                .addData("Left","%.3f", power_left_front)
                .addData("Right", "%.3f", power_right_front);
        robot.opMode.telemetry.addLine(" Rear Wheel Power ")
                .addData("Left", "%.3f", power_left_back)
                .addData("Right","%.3f",  power_right_back);

        // Code for the arm and dics launching mechanisms
        if (robot.opMode.gamepad1.right_trigger == 1) {

        }
    }

    public void AutonomousDrive(double leftFront, double leftBack,
                                double rightFront, double rightBack,
                                double motor_power){
        int new_left_front;
        int new_left_back;
        int new_right_front;
        int new_right_back_target;

        // Determine new target position, and pass to motor controller
        new_left_front = left_front.getCurrentPosition() + (int) (COUNTS_PER_INCH_DOUBLE * leftFront);
        new_left_back = left_back.getCurrentPosition() + (int) (COUNTS_PER_INCH_DOUBLE * leftBack);
        new_right_front = right_front.getCurrentPosition() + (int) (COUNTS_PER_INCH_DOUBLE * rightFront);
        new_right_back_target = right_back.getCurrentPosition() + (int) (COUNTS_PER_INCH_DOUBLE * rightBack);
        right_back.setTargetPosition(new_left_front);
        left_back.setTargetPosition(new_left_back);
        right_front.setTargetPosition(new_right_front);
        left_front.setTargetPosition(new_right_back_target);

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
    public void gyroDrive ( double left_front_power, double left_back_power,
                            double right_front_power, double right_back_power,
                            double motor_power,
                            double angle) {

        int     new_left_front_target;
        int     new_right_front_target;
        int     new_left_back_target;
        int     new_right_back_target;
        int     left_front_counts;
        int     left_back_counts;
        int     right_front_counts;
        int     right_back_counts;
        double  max;
        double  error;
        double  steer;
        double  left_front_speed;
        double  right_front_speed;
        double  left_back_speed;
        double  right_back_speed;

        // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            left_front_counts      = (int)(left_front_power * COUNTS_PER_INCH_DOUBLE);
            left_back_counts       = (int)(left_back_power * COUNTS_PER_INCH_DOUBLE);
            right_back_counts      = (int)(right_back_power * COUNTS_PER_INCH_DOUBLE);
            right_front_counts     = (int)(right_front_power * COUNTS_PER_INCH_DOUBLE);
            new_left_front_target  = left_front.getCurrentPosition() + left_front_counts;
            new_right_front_target = right_front.getCurrentPosition() + right_front_counts;
            new_left_back_target   = left_back.getCurrentPosition() + left_back_counts;
            new_right_back_target  = right_back.getCurrentPosition() + right_back_counts;

            // Set Target and Turn On RUN_TO_POSITION
            left_front.setTargetPosition(new_left_front_target);
            right_front.setTargetPosition(new_right_front_target);
            left_back.setTargetPosition(new_left_back_target);
            right_back.setTargetPosition(new_right_back_target);

            left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            motor_power = Range.clip(Math.abs(motor_power), 0.0, 1.0);
            left_front.setPower(motor_power);
            right_front.setPower(motor_power);
            left_back.setPower(motor_power);
            right_back.setPower(motor_power);

            // keep looping while we are still active, and BOTH motors are running.
            while (left_front.isBusy() && right_front.isBusy() && left_back.isBusy() && right_back.isBusy()) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed

                left_front_speed  = motor_power - steer;
                right_front_speed = motor_power + steer;
                left_back_speed   = motor_power - steer;
                right_back_speed  = motor_power + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(left_front_speed), Math.max(Math.abs(right_front_speed), Math.max(Math.abs(left_back_speed), Math.abs(right_back_speed))));
                if (max > 1.0)
                {
                    left_front_speed /= max;
                    right_front_speed /= max;
                    left_back_speed /= max;
                    right_back_speed /= max;
                }

                left_front.setPower(motor_power);
                right_front.setPower(motor_power);
                left_back.setPower(motor_power);
                right_back.setPower(motor_power);

                // Display drive status for the driver.
                robot.opMode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                robot.opMode.telemetry.addData("Target",  "%7d:%7d",      new_left_front_target,  new_right_front_target, new_left_back_target, new_right_back_target);
                robot.opMode.telemetry.addData("Actual",  "%7d:%7d",      left_front.getCurrentPosition(), right_front.getCurrentPosition(), left_back.getCurrentPosition(), right_back.getCurrentPosition());
                robot.opMode.telemetry.addData("Speed",   "%5.2f:%5.2f",  left_front_speed, right_front_speed, left_back_speed, right_back_speed);
                robot.opMode.telemetry.update();
            }

            // Stop all motion;
            left_front.setPower(0);
            right_front.setPower(0);

            // Turn off RUN_TO_POSITION
            left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    }
    public void gyroTurn( double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            robot.opMode.telemetry.update();
        }
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double   leftSpeed;
        double   rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer      = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget   = true;
        }
        else {
            steer       = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        left_front.setPower(leftSpeed);
        right_front.setPower(rightSpeed);
        left_back.setPower(leftSpeed);

        // Display it for the driver.
        robot.opMode.telemetry.addData("Target", "%5.2f", angle);
        robot.opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        robot.opMode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
