package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ExRingDetector extends LinearOpMode {
    RectDetector rectDetector;

    public void initOpMode() {
        rectDetector = new RectDetector(hardwareMap);
        rectDetector.init();
    }

    public void runOpMode() {
        RectDetector.Stack stack = rectDetector.getStack();

        switch (stack) {
            case ZERO:
                break;
            case ONE:
                break;
            case FOUR:
                break;
            default:
                break;
        }
    }
}
