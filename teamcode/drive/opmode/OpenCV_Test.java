package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvTracker;
import org.openftc.easyopencv.OpenCvTrackerApiPipeline;

/**
 * In this sample, we demonstrate how to use the {@link OpenCvTrackerApiPipeline()}
 * class to run multiple {@link OpenCvTracker} instances on each frame from the camera.
 */
@Autonomous
public class OpenCV_Test extends LinearOpMode
{
    OpenCvCamera phoneCam;
    OpenCvTrackerApiPipeline trackerApiPipeline;
    UselessColorBoxDrawingTracker tracker1, tracker2, tracker3;

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCV,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        /**
         * Create an instance of the {@link OpenCvTrackerApiPipeline}
         * pipeline (included with EasyOpenCV), and tell the camera
         * to use it.
         */
        trackerApiPipeline = new OpenCvTrackerApiPipeline();
        phoneCam.setPipeline(trackerApiPipeline);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        /*
         * Create some trackers we want to run
         */
        tracker1 = new UselessColorBoxDrawingTracker(new Scalar(255, 0, 0));
        tracker2 = new UselessColorBoxDrawingTracker(new Scalar(0, 255, 0));
        tracker3 = new UselessColorBoxDrawingTracker(new Scalar(0, 0, 255));

        /*
         * Add those trackers to the pipeline. All trackers added to the
         * trackerApiPipeline will be run upon receipt of a frame from the
         * camera. Note: the trackerApiPipeline will handle switching
         * the viewport view on tap between the output of each of the trackers
         * for you.
         */
        trackerApiPipeline.addTracker(tracker1);
        trackerApiPipeline.addTracker(tracker2);
        trackerApiPipeline.addTracker(tracker3);

        waitForStart();

        while (opModeIsActive())
        {
            /*
             * If you later want to stop running a tracker on each frame,
             * you can remove it from the trackerApiPipeline like so:
             */
            //trackerApiPipeline.removeTracker(tracker1);

            sleep(100);
        }
    }

    class UselessColorBoxDrawingTracker extends OpenCvTracker
    {
        Scalar color;

        UselessColorBoxDrawingTracker(Scalar color)
        {
            this.color = color;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    color, 4);

            return input;
        }
    }
}