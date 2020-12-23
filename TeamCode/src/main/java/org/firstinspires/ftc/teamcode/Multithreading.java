package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Auto.Auto;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

import java.util.List;

@TeleOp(name = "Multithreading")
public class Multithreading extends LinearOpMode {
    private Robot robot;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        Thread driveThread = new DriveThread();
        // start thread to have robot move while vision is being initialized

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();

        driveThread.start();

        int value = 1;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Main thread ",value);
                telemetry.update();
                value += 2;
//                if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
//
//                        // step through the list of recognitions and display boundary info.
//                        int i = 0;
//                        for (Recognition recognition : updatedRecognitions) {
//                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                    recognition.getLeft(), recognition.getTop());
//                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                    recognition.getRight(), recognition.getBottom());
//                        }
//                        telemetry.update();
//                    }
//                }
            }

        }

        driveThread.interrupt();
    }

    class DriveThread extends Thread {
        public DriveThread() {
            this.setName("DriveThread");

            telemetry.addData("DriveThread ", this.getName());
            telemetry.update();
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            telemetry.addData("Starting thread ", this.getName());
            int evenValue = 0;
            try {
//                while (!isInterrupted()) {
//                    // we record the Y values in the main class to make showing them in telemetry
//                    // easier.
//
//                    robot.leftStickY = gamepad1.left_stick_y * -1;
//                    robot.rightStickY = gamepad1.right_stick_y * -1;
//
////                    robot.frontLeftDriveMotor.setPower(Range.clip(leftY, -1.0, 1.0));
////                    rightMotor.setPower(Range.clip(rightY, -1.0, 1.0));
//
//                    idle();
//                }

                while (!isInterrupted()) {
                    telemetry.addData("Running thread ",evenValue);
                    telemetry.update();
                }

                evenValue += 2;
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.
            catch (Exception e) {
//                e.printStackTrace(Logging.logPrintStream);
            }

//            Logging.log("end of thread %s", this.getName());
        }
    }
}