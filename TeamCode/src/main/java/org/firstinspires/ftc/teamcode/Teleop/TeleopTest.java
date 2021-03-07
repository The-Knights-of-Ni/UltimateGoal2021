package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.io.IOException;


@TeleOp(name="TeleopTest")
public class TeleopTest extends LinearOpMode{

    public final double TICKS_PER_REV = 537.6;
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

            double speed1 = robot.launch1.getVelocity(AngleUnit.DEGREES) * 60 / 360;
            double speed2 = robot.launch2.getVelocity(AngleUnit.DEGREES) * 60 / 360;

            telemetry.addData("Launch 1 RPM: ", speed1);
            telemetry.addData("Launch 2 RPM: ", speed2);

//            boolean xB = robot.xButton;
//            boolean yB = robot.yButton;
//
//            if (xB) {
//                robot.elevator1.setPower(0.2);
//                robot.elevator2.setPower(0.2);
//            } else if (yB){
//                robot.elevator1.setPower(-0.2);
//                robot.elevator2.setPower(-0.2);
//            } else {
//                robot.elevator1.setPower(0.0);
//                robot.elevator2.setPower(0.0);
//            }

            telemetry.update();
        }
    }

}
