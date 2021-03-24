package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.io.IOException;

/**
 * Created by Andrew Chiang on 3/23/2021
 */

@TeleOp(name = "TeleopMark2")
public class TeleopMark2 extends LinearOpMode {
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
    private double robotAngle;
    private boolean visionEnabled = false;
    private boolean wobbleClawControlDigital = true;
    private boolean wobbleClawDeployed = false;
    private boolean wobbleClawOpen = false;

    private boolean isIntakeOn = false;
    private boolean isLaunchOn = false;
    private boolean isIntakeToElevatorOpen = false;
    private boolean isLauncherFeeder = false;







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

//        robot.initServosTeleop();

        telemetry.clearAll();
        timeCurrent = timer.nanoseconds();
        timePre = timeCurrent;

        while(opModeIsActive()) {

            // Get gamepad inputs
            robot.getGamePadInputs();

            // Get the current time
            timeCurrent = timer.nanoseconds();
            deltaT = timeCurrent - timePre;
            timePre = timeCurrent;

            // Drive the motors
            double[] motorPowers;
            robotAngle = robot.imu.getAngularOrientation().firstAngle;

            motorPowers = robot.drive.calcMotorPowers(robot.leftStickX, robot.leftStickY, robot.rightStickX);
            robot.drive.setDrivePowers(motorPowers);

            // reset drive motor encoders
            if (robot.yButton && !robot.isyButtonPressedPrev) {
                robot.drive.resetDriveMotorEncoders();
            }


//            //Open and close claw
//            if (wobbleClawControlDigital) {
//                if (robot.bumperRight2 && !robot.isrBumper2PressedPrev) { // toggle main claw arm deploy mode
//                    robot.control.openWobbleClaw();
//                        robot.isrBumper2PressedPrev = true;
//                    wobbleClawOpen = true;
//                }
//                else
//                    robot.isrBumper2PressedPrev = false;
//            }
//            if ((robot.triggerRight2 > 0.5) && (robot.triggerLeft2 < 0.5)){
//                robot.control.closeWobbleClaw();
//                wobbleClawOpen = false;
//            }
//
//            //Retract and deploy arm
//            if (wobbleClawControlDigital) {
//                if (robot.bumperLeft2 && !robot.islBumper2PressedPrev) { // toggle main claw arm deploy mode
//                    robot.control.deployWobble();
//                    wobbleClawDeployed = true;
//                }
//            }
//            if ((robot.triggerLeft2 > 0.5) && (robot.triggerRight2 < 0.5)){
//                robot.control.retractWobble();
//                wobbleClawDeployed = false;
//            }

            //Toggle intake regular
            if (robot.aButton && !robot.isaButtonPressedPrev){
                if(isIntakeOn){
                    robot.control.setIntake(false);
                    isIntakeOn = false;
                }
                else{
                    robot.control.setIntake(true);
                    isIntakeOn = true;
                }
            }

            //Toggle intake REVERSE
            if (robot.bButton && !robot.isbButtonPressedPrev){
                if(isIntakeOn){
                    robot.control.setIntakeReverse(false);
                    isIntakeOn = false;
                }
                else{
                    robot.control.setIntakeReverse(true);
                    isIntakeOn = true;
                }
            }

            //Toggle launcher
            if (robot.xButton && !robot.isxButtonPressedPrev){
                if(isLaunchOn) {
                    robot.control.setLaunch(false);
                    isLaunchOn = false;
                }
                else{
                    robot.control.setLaunch(true);
                    isLaunchOn = true;
                }

            }

//            int currentPositions[] = robot.drive.getCurrentPositions();
//            telemetry.addData("position", "fl %d, fr %d, rl %d, rr %d",
//                    currentPositions[0], currentPositions[1], currentPositions[2], currentPositions[3]);
            telemetry.update();
        }
    }

}