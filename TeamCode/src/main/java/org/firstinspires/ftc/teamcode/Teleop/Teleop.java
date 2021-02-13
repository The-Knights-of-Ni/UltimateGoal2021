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

//    enum MainClawState {
//        CLOSE,
//        OPEN,
//        WIDEOPEN,
//    }

    private Prospective prospectiveMode = Prospective.ROBOT;
    private double robotAngle;
    private boolean visionEnabled = false;
    private boolean wobbleClawControlDigital = true;
    private boolean wobbleClawDeployed = false;



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
//                robot.vision.vuMarkScan();
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

            if (wobbleClawControlDigital) {
                if (robot.bumperRight2 && !robot.isrBumper2PressedPrev) { // toggle main claw arm deploy mode
                    if (wobbleClawDeployed) {
                        robot.control.retractWobbleClaw();
                        wobbleClawDeployed = false;
                    }
                    else {
                        robot.control.setWobbleAngle(robot.control.getWobbleArmTargetAngle());
                        wobbleClawDeployed = true;
                    }
                }
            }
            if ((robot.triggerLeft2 > 0.5) && (robot.triggerRight2 < 0.5)) {
                robot.control.openWobbleClaw();
            }
            else if ((robot.triggerRight2 > 0.5) && (robot.triggerLeft2 < 0.5)){
                robot.control.closeWobbleClaw();
            }
            if (robot.aButton && !robot.isaButtonPressedPrev){
                //
                robot.control.setIntake(true);
            }
            else if (robot.aButton){
                robot.control.setIntake(false);
            }
            if (robot.bButton && !robot.isbButtonPressedPrev){
                robot.control.setLaunch(true);
            }
            else if (robot.bButton){
                robot.control.setLaunch(false);
            }

            telemetry.addData("Drive Mode ", prospectiveMode.toString());
            telemetry.addData("robot angle ", robotAngle);

            int currentPositions[] = robot.drive.getCurrentPositions();
            telemetry.addData("position", "fl %d, fr %d, rl %d, rr %d",
                    currentPositions[0], currentPositions[1], currentPositions[2], currentPositions[3]);
            telemetry.update();
        }
    }

}