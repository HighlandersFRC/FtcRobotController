

package org.firstinspires.ftc.teamcode.OpenCv;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@TeleOp(name = "Concept: TensorFlow Object Detection White Pixel", group = "Concept")
//@Disabled
public class TensorFlowObjectDetection1 extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private static final String TFOD_MODEL_ASSET = "model_20231111_163542.tflite";
    private static final String[] LABELS = {
            "BLUECUBE",
            "REDCUBE",
            "WHITEPIXEL",
            "PURPLEPIXEL",
            "GREENPIXEL",
            "YELLOWPIXEL"
    };



    @Override
    public void runOpMode() {

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
            //    if (gamepad1.dpad_down) {
              //      visionPortal.stopStreaming();
                //} else if (gamepad1.dpad_up) {
                  //  visionPortal.resumeStreaming();
               //}

                // Share the CPU.
                sleep(20);
            }
        }


        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

            .setModelAssetName(TFOD_MODEL_ASSET)
           // .setModelFileName(TFOD_MODEL_ASSET) //if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            //.setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
            .setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            .setIsModelQuantized(true)
            .setModelInputSize(360)
            //.setModelAspectRatio(16.0 / 9.0)

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "webcam1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);


        builder.addProcessor(tfod);


        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);


        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection recognitions.

     * @return
     */
    private String telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            float x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            float y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

if (x < 250) {
    telemetry.addData("Location", "Left");
    return "Left";
}
if (x > 430) {
    telemetry.addData("Location", "Right");
    return "Right";
}
if (x > 250 && x < 430) {
    telemetry.addData("Location", "Middle");
    return "Middle";
}

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

        return null;

    }   // end method telemetryTfod()

}   // end class