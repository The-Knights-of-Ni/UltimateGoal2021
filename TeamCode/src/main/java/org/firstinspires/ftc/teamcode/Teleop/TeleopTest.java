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
    private static final int        TILT_TABLE_SIZE                     = 83;
    private static final double[]   TILT_TABLE = {
            0.0, 0, 0.6, 88, 1.5, 131, 2.2, 160, 2.9, 200, 3.4, 223, 4.1, 257, 5.0, 301, 5.8, 350, 6.7, 392,
            7.3, 432, 8.2, 479, 9.1, 535, 9.7, 571, 10.5, 617, 11.2, 658, 11.7, 686, 12.2, 716, 12.7, 748, 13.2, 781,
            13.6, 810, 14.1, 838, 14.7, 880, 15.2, 918, 15.8, 951, 16.1, 972, 16.9, 1023, 17.9, 1093, 18.5, 1131, 18.9, 1158,
            19.5, 1205, 20.2, 1252, 20.9, 1300, 21.3, 1333, 21.9, 1376, 22.7, 1428, 23.3, 1473, 23.9, 1518, 25.0, 1602, 25.9, 1666,
            26.9, 1736, 27.8, 1801, 28.5, 1854, 29.5, 1932, 30.3, 1997, 31.2, 2067, 32.0, 2120, 33.0, 2199, 34.0, 2269, 34.7, 2336,
            35.5, 2391, 36.5, 2470, 37.2, 2534, 38.2, 2608, 38.9, 2661, 40.1, 2750, 40.8, 2812, 41.4, 2862, 42.2, 2923, 43.1, 3000,
            44.0, 3060, 45.1, 3152, 45.9, 3210, 46.7, 3284, 47.8, 3372, 48.7, 3444, 49.4, 3501, 50.5, 3584, 51.0, 3630, 51.9, 3700,
            53.3, 3804, 54.4, 3893, 55.5, 3987, 57.0, 4106, 58.1, 4186, 59.6, 4301, 61.0, 4414, 62.0, 4491, 63.0, 4564, 63.7, 4615,
            64.7, 4688, 65.9, 4771, 67.5, 4889 };

    /**
     * The CLAW_ARM_TILT_TABLE is a lookup table for mapping the claw arm angle to the claw arm servo
     * The CLAW_ARM_TILT_TABLE_SIZE records the number of entries in the CLAW_ARM_TILT_TABLE
     * The CLAW_ARM_TILT_TABLE consists of CLAW_ARM_TILT_TABLE_SIZE pairs of data. Each pair is (tilt angle, arm tilt servo).
     */
    private static final int        CLAW_ARM_TILT_TABLE_SIZE                     = 24;
    private static final double[]   CLAW_ARM_TILT_TABLE = {
            -183.0, 0.01, -180.0, 0.04, -70.2, 0.416, -30.8, 0.563, -12.2, 0.634, -5.8, 0.658, -2.9, 0.679, 0.2, 0.692, 4.6, 0.703, 6.1, 0.716,
            11.0, 0.736, 15.2, 0.743, 18.2, 0.763, 23.2, 0.782, 29.0, 0.804, 33.9, 0.825, 40.8, 0.851, 46.6, 0.879, 51.0, 0.894, 57.4, 0.921,
            62.5, 0.941, 70.1, 0.971, 75.6, 0.993, 77.6, 1.0 };

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
    public double getWobbleArmTargetAngle() {
        return 0.0;
    }
    public double wobbleGoalArmAngleToPos(double angle){
        int lowerIndex, upperIndex;
        int i = 1;
        double servoTarget;
        while ((i < CLAW_ARM_TILT_TABLE_SIZE) && (CLAW_ARM_TILT_TABLE[i*2] < angle)) {
            ++i;
        }
        upperIndex = i;
        lowerIndex = i-1;
        servoTarget = CLAW_ARM_TILT_TABLE[lowerIndex*2+1] +
                (CLAW_ARM_TILT_TABLE[upperIndex*2+1]-CLAW_ARM_TILT_TABLE[lowerIndex*2+1])*(angle-CLAW_ARM_TILT_TABLE[lowerIndex*2])
                        /(CLAW_ARM_TILT_TABLE[upperIndex*2]-CLAW_ARM_TILT_TABLE[lowerIndex*2]);
        return servoTarget;
    }
    public void setWobbleAngle(double angle){
        robot.wobbleGoalArm.setPosition(this.wobbleGoalArmAngleToPos(angle));
    }
    public void retractWobbleClaw(){
        setWobbleAngle(-180);
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

            double speed1 = robot.launch1.getVelocity(AngleUnit.DEGREES);
            double speed2 = robot.launch2.getVelocity(AngleUnit.DEGREES);

            telemetry.addData("Launch 1 RPM: ", speed1);
            telemetry.addData("Launch 2 RPM: ", speed2);
            if (wobbleClawControlDigital) {
                if (robot.bumperRight2 && !robot.isrBumper2PressedPrev) { // toggle main claw arm deploy mode
                    if (wobbleClawDeployed) {
                        robot.control.retractWobble();
                        wobbleClawDeployed = false;
                    }
                    else {
                        robot.control.deployWobble();
                        wobbleClawDeployed = true;
                    }
                }
            }
            if ((robot.triggerLeft2 > 0.5) && (robot.triggerRight2 < 0.5)) {
                robot.control.closeWobbleClaw();
            }
            else if ((robot.triggerRight2 > 0.5) && (robot.triggerLeft2 < 0.5)){
                robot.control.openWobbleClaw();
            }

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
