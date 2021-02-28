package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

@TeleOp(name = "Tower Detection CV")
public class TowerDetectionCV extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    TowerDetector detector = new TowerDetector(false);

    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        TowerDetector pipeline = new TowerDetector(false);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Goal found: ", pipeline.goalFound());
        }
    }

    class TowerDetector extends OpenCvPipeline {
        private boolean isBlue;
        private boolean goalFound;

        /**
         *
         * @param isBlue are we blue alliance or red alliance?
         */
        public TowerDetector(boolean isBlue) {
            this.isBlue = isBlue;
            this.goalFound = false;
        }

        @Override
        public Mat processFrame(Mat input) {

            Mat mat = new Mat();
            Mat matY = new Mat();
            Mat matCr = new Mat();
            Mat matCb = new Mat();

            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(mat, matY, 0);
            Core.extractChannel(mat, matCr, 1);
            Core.extractChannel(mat, matCb, 2);
            // matY frame is the Y frame
            // matCr frame is the Cr frame
            // matCb frame is the Cb frame

            // if something is wrong, we assume there's no skystone
            if (mat.empty()) {
                return input;
            }

            Mat thresh = new Mat();

            if(isBlue)
                Imgproc.threshold(matCb, thresh, 150, 255, Imgproc.THRESH_BINARY);
            else
                Imgproc.threshold(matCr, thresh, 150, 255, Imgproc.THRESH_BINARY);

            return thresh; // return the mat with rectangles drawn
        }

        public boolean goalFound() {
            return goalFound();
        }
    }

}
