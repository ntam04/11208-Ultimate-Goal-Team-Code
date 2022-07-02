package org.firstinspires.ftc.teamcode.vision;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Team Marker Detector")
@Disabled
public class TeamMarkerDetectorProgram extends LinearOpMode {

    OpenCvCamera Robot;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Robot = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        TeamMarkerDetectorBlue detector = new TeamMarkerDetectorBlue(telemetry);
        Robot.setPipeline(detector);

        Robot.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Robot.startStreaming(320,176, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        switch (detector.getLocation()) {
            case LEFT:
                // ...
                break;
            case CENTER:
                // ...
                break;
            case RIGHT:
                // ...
                break;
        }

        Robot.stopStreaming();
    }
}
