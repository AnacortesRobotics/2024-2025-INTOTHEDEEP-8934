package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.OpModesAndRobots.autoBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;


import java.util.List;
import java.util.concurrent.TimeUnit;

public class visionManager {

    static final private int DEFAULT_ZONE_IF_VISION_FAILS = 3;
    VisionPortal visionPortal;
    propProcessor visProcessor;
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private Scalar colorUpperBound;
    private Scalar colorLowerBound;

    public visionManager(autoBase.ALLIANCE thisAlliance, WebcamName thisWebcam)
    {
        if (thisAlliance == autoBase.ALLIANCE.RED)
        {
            colorLowerBound = new Scalar(0, 150, 50);
            colorUpperBound = new Scalar(20, 255, 200);
        }
        else {
            colorLowerBound = new Scalar(106, 150, 50);
            colorUpperBound = new Scalar(146, 255, 200);
        }

        visProcessor = new propProcessor(colorLowerBound, colorUpperBound);

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        VisionPortal.Builder visPortalBuilder;
        visPortalBuilder = new VisionPortal.Builder();
        visPortalBuilder.setCamera(thisWebcam);
        // IMPORTANT: both processors need to be added to the same vision portal.
        // Here we use the Builder pattern as recommended by the VisionPortal javadoc.
        visPortalBuilder.addProcessor(aprilTag);
        visPortalBuilder.addProcessor(visProcessor);
        visionPortal = visPortalBuilder.build();
        visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
                roboNap(20);
        }
    }

    public int getDetectedSpikeMark()
    {
        int detectedSpikeMark = -1;
        // keep scanning until we find something
        int numTries = 0;
        final int maxTries = 5;
        while (numTries < maxTries) {
            int newPropLocation = visProcessor.getDetectedSpikeMark();
            if (newPropLocation > 0) {
                detectedSpikeMark = newPropLocation;
                break;
            }
            numTries++;
            roboNap(100);
        }

        // default to zone 3 if we got this far and we still don't know where to go.
        if (detectedSpikeMark < 1) {
            detectedSpikeMark = DEFAULT_ZONE_IF_VISION_FAILS;
        }
        return detectedSpikeMark;
    }

    public AprilTagDetection getDetectedAprilTag(int desiredDetectionId)
    {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection targetAprilTag = null;
        if (currentDetections != null) {
            // Step through the list of detected tags and look for a matching tag
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == desiredDetectionId) {
                        // Yes, we want to use this tag.
                        targetAprilTag = detection;
                        break; // don't look further
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        // telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    // telemetry.addData("Unknown", "could not parse any apriltags matching criteria");
                }
            }

        }
        return targetAprilTag;
    }

    // FIXME: is this necessary?
    public void enableAprilTagProcessor() {
        visionPortal.setProcessorEnabled(visProcessor, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
        setManualExposure(1, 200);  // Use low exposure time to reduce motion blur
    }
    private void setManualExposure(int exposureMS, int gain) {

        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
            while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
            {
                roboNap(20);
            }
//            telemetry.addData("Camera", "Ready");
        }

        // Set camera controls unless we are stopping.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            roboNap(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        roboNap(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        roboNap(20);
    }
    private void roboNap(int millis)
    {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

}