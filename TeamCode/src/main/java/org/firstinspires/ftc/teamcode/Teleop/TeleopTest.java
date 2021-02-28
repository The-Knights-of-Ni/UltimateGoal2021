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

        telemetry.addData("Wait for start", "");
        telemetry.update();
    }

    public void runOpMode() throws InterruptedException{
        initOpMode();
        waitForStart();
        while (opModeIsActive()){
            robot.getGamePadInputs();
            boolean aB = robot.aButton;
            boolean bB = robot.bButton;

            telemetry.addData("A Button: ", aB);
            telemetry.addData("B Button: ", bB);

            if (aB){
                robot.launch1.setPower(1.0);
                robot.launch2.setPower(1.0);
            }
            if (bB){
                robot.launch1.setPower(0.0);
                robot.launch2.setPower(0.0);
            }
            telemetry.update();
        }
    }

}
