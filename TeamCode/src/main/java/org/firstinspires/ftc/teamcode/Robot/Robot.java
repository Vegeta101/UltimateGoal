package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    MecanumDriveTrain driveTrain;
    ComputerVision computerVision;

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
