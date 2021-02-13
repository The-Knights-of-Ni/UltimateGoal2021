package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.io.IOException;


@TeleOp(name="TeleopTest")
public class TeleopTest extends LinearOpMode{

    private Robot robot;

    double deltaT;
    double timeCurrent;
    double timePre;
    ElapsedTime timer;
    private void initOpMode() {
        //Initialize DC motor objects
        timer = new ElapsedTime();
        try {
            robot = new Robot(this, timer, true);
        } catch (IOException e) {
            e.printStackTrace();
        }
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

    }

    public void runOpMode() throws InterruptedException{
        initOpMode();
        waitForStart();
        while (opModeIsActive()){
            robot.getGamePadInputs();
            if (robot.aButton){
                robot.control.setLaunch(true);
            }
            if (robot.bButton){
                robot.control.setLaunch(false);
            }
        }
    }

}
