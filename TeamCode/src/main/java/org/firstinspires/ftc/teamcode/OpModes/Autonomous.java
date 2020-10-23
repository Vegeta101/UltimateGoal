package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import  com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Robot;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Production")

public class Autonomous extends LinearOpMode {
    Robot robot = new Robot (this);
    private boolean inInitializationState() {
        return (!opModeIsActive() && !isStopRequested());
    }
    @Override
    public void runOpMode(){
        String ringpattern = "";
        robot.getDriveTrain().init();
        robot.getComputerVision().init();
        sleep(500);
        
        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        while (inInitializationState()) {

        }
        if (opModeIsActive()){
            robot.getDriveTrain().AutonomousDrive(10, 10, 10, 10, .5);
            while (opModeIsActive()) {
                ringpattern = robot.getComputerVision().detect();
                sleep(500);
                telemetry.addData("ring patter", ringpattern);
                telemetry.addData("", "----------------------------");
                telemetry.addData("sec", String.format("%.2f", timer.seconds()));

                telemetry.addData(">", "Press Play to Start");
                telemetry.update();
            }
        }

        robot.getComputerVision().shutdown();
    }
}
