package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    public BNO055IMU imu_hub1;
    MecanumDriveTrain driveTrain;
    ComputerVision computerVision;
    ModernRoboticsI2cGyro gyro;

    ElapsedTime autonomusTime;
    final LinearOpMode opMode;

    public Robot (LinearOpMode opMode){
        this.opMode      = opMode;
        this.autonomusTime = new ElapsedTime();
        this.driveTrain = new MecanumDriveTrain(this);
        this.computerVision = new ComputerVision(this);
    }

    public MecanumDriveTrain getDriveTrain(){return this.driveTrain;}
    public ComputerVision getComputerVision() {return this.computerVision;}

}
