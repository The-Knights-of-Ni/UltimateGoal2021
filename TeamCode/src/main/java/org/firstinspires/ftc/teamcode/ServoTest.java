package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;


import java.io.IOException;

/**
 * Created by tarunsingh on 12/5/17.
 * Modified by AndrewC on 1/17/2020.
 */

@TeleOp(name="Servo Test")
public class ServoTest extends LinearOpMode {
    private static final int targetPosition = 315;
    private static final double maxPower = 0;
    private static final float mmPerInch        = 25.4f;

    private Robot robot;

    double timePre;
    double timeCurrent;
    ElapsedTime timer;

    public void initOpMode() throws IOException {
        timer = new ElapsedTime();
        this.robot = new Robot(this, timer, true);

        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        telemetry.addData("Wait for start", "");
        telemetry.update();

    }
    public void runOpMode() {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }
        //robot.initServosAuto();
        telemetry.clearAll();
//        telemetry.addLine("Wait For Start");
//        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            //Get gamepad inputs
            robot.getGamePadInputs();

            //Get the current time
            timeCurrent = timer.nanoseconds();

            // test main arm servos
            if (robot.triggerLeft2 > 0.5) {
                robot.control.modifyServo(robot.wobbleClaw, 0.005);
            }
            else if (robot.triggerLeft2 > 0.1){
                robot.control.modifyServo(robot.wobbleClaw, 0.001);
            }
            else if (robot.triggerRight2 > 0.5){
                robot.control.modifyServo(robot.wobbleClaw, -0.005);
            }
            else if (robot.triggerRight2 > 0.1){
                robot.control.modifyServo(robot.wobbleClaw, -0.001);
            }


            telemetry.addData("Wobble Claw      ", "%.3f", robot.wobbleClaw.getPosition());
            //telemetry.addData("Wobble Goal Arm  ", "%.3f", robot.wobbleGoalArm.getPosition());

            telemetry.update();
            sleep(100);
        }

    }
}