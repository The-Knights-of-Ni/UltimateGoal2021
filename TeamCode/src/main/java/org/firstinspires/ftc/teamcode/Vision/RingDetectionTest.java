package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.io.IOException;

@TeleOp(name="Vision Test")
public class RingDetectionTest extends LinearOpMode {
    //Declare DC motor objects
    private Robot robot;

    public void initOpMode() throws IOException {
        ElapsedTime timer = new ElapsedTime();
        this.robot = new Robot(this, timer);

    }

    public void runOpMode() throws InterruptedException {
        try {
            initOpMode();
        } catch (IOException e) {
            e.printStackTrace();
        }
        waitForStart();

        robot.vision.vuMarkScan();
    }
}
