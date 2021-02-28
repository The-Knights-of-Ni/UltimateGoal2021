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
            // "Mat" stands for matrix, which is basically the image that the detector will process
            // the input matrix is the image coming from the camera
            // the function will return a matrix to be drawn on your phone's screen

            // The detector detects regular stones. The camera fits two stones.
            // If it finds one regular stone then the other must be the skystone.
            // If both are regular stones, it returns NONE to tell the robot to keep looking

            // Make a working copy of the input matrix in HSV
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

            // We create a HSV range for yellow to detect regular stones
            // NOTE: In OpenCV's implementation,
            // Hue values are half the real value
            Mat thresh = new Mat();

            Imgproc.threshold(matCr, thresh, 120, 255, Imgproc.THRESH_BINARY);



            // We'll get a black and white image. The white regions represent the regular stones.
            // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
            //Core.inRange(mat, lowHSV, highHSV, thresh);

            // Use Canny Edge Detection to find edges
            // you might have to tune the thresholds for hysteresis
            Mat edges = new Mat();
            Imgproc.Canny(thresh, edges, 100, 300);

            // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
            // Oftentimes the edges are disconnected. findContours connects these edges.
            // We then find the bounding rectangles of those contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            }

            if(contours.size() > 0)
                goalFound = true;

            return matY; // return the mat with rectangles drawn
        }

        public boolean goalFound() {
            return goalFound();
        }
    }

}
