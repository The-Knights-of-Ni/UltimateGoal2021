import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class UGContourRingPipelineJavaExample extends LinearOpMode {
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = ""; // insert webcam name from configuration if using webcam

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = this
            .hardwareMap
            .appContext
            .getResources().getIdentifier(
                    "cameraMonitorViewId",
                    "id",
                    hardwareMap.appContext.getPackageName()
            );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        }

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        UGContourRingPipeline.Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        UGContourRingPipeline.Config.setHORIZON(HORIZON);

        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        waitForStart();

        while (opModeIsActive()) {
            String height = "[HEIGHT]" + " " + pipeline.getHeight();
            telemetry.addData("[Ring Stack] >>", height);
            telemetry.update();
        }
    }
}
import com.arcrobotics.ftclib.vision.UGContourRingPipeline
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.*

class UGContourRingPipelineKtExample: LinearOpMode() {
    companion object {
        val CAMERA_WIDTH = 320 // width  of wanted camera resolution
        val CAMERA_HEIGHT = 240 // height of wanted camera resolution

        val HORIZON = 100 // horizon value to tune

        val DEBUG = false // if debug is wanted, change to true

        val USING_WEBCAM = false // change to true if using webcam
        val WEBCAM_NAME = "" // insert webcam name from configuration if using webcam
    }

    private lateinit var pipeline: UGContourRingPipeline
    private lateinit var camera: OpenCvCamera

    private var cameraMonitorViewId: Int = -1

    private fun configurePhoneCamera(): OpenCvInternalCamera = OpenCvCameraFactory.getInstance()
                .createInternalCamera(
                        OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId,
                )

    private fun configureWebCam(): OpenCvWebcam = OpenCvCameraFactory.getInstance().createWebcam(
                        hardwareMap.get(
                                WebcamName::class.java,
                                WEBCAM_NAME
                        ),
                        cameraMonitorViewId,
                )

    override fun runOpMode() {
        cameraMonitorViewId = hardwareMap
            .appContext
            .resources
            .getIdentifier(
                    "cameraMonitorViewId",
                    "id",
                    hardwareMap.appContext.packageName,
            )
        camera = if (USING_WEBCAM) configureWebCam() else configurePhoneCamera()
        
        camera.setPipeline(UGContourRingPipeline(telemetry, DEBUG).apply { pipeline = this })

        UGContourRingPipeline.Config.CAMERA_WIDTH = CAMERA_WIDTH

        UGContourRingPipeline.Config.HORIZON = HORIZON

        camera.openCameraDeviceAsync {
            camera.startStreaming(
                    CAMERA_WIDTH,
                    CAMERA_HEIGHT,
                    OpenCvCameraRotation.UPRIGHT,
            )
        }

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("[Ring Stack] >>", "[HEIGHT] ${pipeline.height}")
            telemetry.update()
        }
    }

}
