package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.io.IOException;

/**
 * Created by PaigeYeung on 10/17/2020
 */

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    //Declare DC motor objects
    private Robot robot;

    double deltaT;
    double timeCurrent;
    double timePre;
    ElapsedTime timer;

    enum Prospective {
        ROBOT,
        DRIVER,
    }

    enum MainClawState {
        CLOSE,
        OPEN,
        WIDEOPEN,
    }

    private Prospective prospectiveMode = Prospective.ROBOT;
    private double robotAngle;
    private boolean visionEnabled = false;

    private double turretAngle; // for turret

    private void initOpMode() {
        //Initialize DC motor objects
        timer = new ElapsedTime();
        if (visionEnabled) {
            // visionMode 4: backWebcam is initialized for Vuforia and armWebcam is initialized for OpenCV
            try {
                robot = new Robot(this, timer, 4);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        else {
            try {
                robot = new Robot(this, timer);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        telemetry.addData("Wait for start", "");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initOpMode();
        waitForStart();

        // call initServosTeleop() after running Auto program
        robot.initServosTeleop();
        // call initServosAuto() if testing Teleop stand-alone
//        robot.initServosAuto();

        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;
//        if (visionEnabled) {
//            robot.vision.getTargetsSkyStone().activate();
//        }

//      mainClawState = MainClawState.CLOSE;
        while(opModeIsActive()) {

            // Get gamepad inputs
            robot.getGamePadInputs();

            // Get the current time
            timeCurrent = timer.nanoseconds();
            deltaT = timeCurrent - timePre;
            timePre = timeCurrent;

            if (visionEnabled) {
                robot.vision.vuMarkScan();
            }

            // Drive the motors
            double[] motorPowers;
            robotAngle = robot.imu.getAngularOrientation().firstAngle;
            if (prospectiveMode == Prospective.ROBOT) {
                motorPowers = robot.drive.calcMotorPowers(robot.leftStickX, robot.leftStickY, robot.rightStickX);
            }
            else {  // DRIVER prospective mode
                // Get robot angle
                double relativeX = robot.leftStickX * Math.cos(robotAngle*Math.PI/180.0) + robot.leftStickY * Math.sin(robotAngle*Math.PI/180.0);
                double relativeY = -robot.leftStickX * Math.sin(robotAngle*Math.PI/180.0) + robot.leftStickY * Math.cos(robotAngle*Math.PI/180.0);
                motorPowers = robot.drive.calcMotorPowers(relativeX, relativeY, robot.rightStickX);
            }
            robot.drive.setDrivePowers(motorPowers);

            // reset drive motor encoders
            if (robot.yButton && !robot.isyButtonPressedPrev) {
                robot.drive.resetDriveMotorEncoders();
            }

            // toggle drive prospective mode
            if (robot.xButton && !robot.isxButtonPressedPrev) {
                if (prospectiveMode == Prospective.DRIVER) {
                    prospectiveMode = Prospective.ROBOT;
                }
                else {
                    prospectiveMode = Prospective.DRIVER;
                }
            }

            telemetry.addData("Drive Mode ", prospectiveMode.toString());
            telemetry.addData("robot angle ", robotAngle);
            telemetry.addData("turret angle ", turretAngle);

            int currentPositions[] = robot.drive.getCurrentPositions();
            telemetry.addData("position", "fl %d, fr %d, rl %d, rr %d",
                    currentPositions[0], currentPositions[1], currentPositions[2], currentPositions[3]);
            telemetry.update();
        }
    }

}