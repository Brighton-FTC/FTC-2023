package org.firstinspires.ftc.teamcode.CV;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

// Test log 15/3/23:
//Figure out why robot can only drive straight when it the velocity reads that both motors are going
//in the opposite direction based on their velocities, implying that the robot cannot turn.
//Perhaphs shift to a turning mechanism where only 1 of the robot's wheels are turning
// Test og 19/3/23:
//fixed the robot turning issue, now resolve why the robot decides to turn left and right in a constant loop
//Theory 1, make the robot turn at extremely reduced speeds
@Autonomous(name="Basic Test Auto CV", group="CV tests")
public class TestAutoCV extends AutonomousLinearModeBase {
    int width = 320;
    int height = 240;
    // store as variable here so we can access the location

    OpenCvCamera phoneCam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        ColorDetector detector = new ColorDetector(telemetry);
        phoneCam.setPipeline(detector);
        // Connect to the camera
        phoneCam.openCameraDevice();
    // Use the SkystoneDetector pipeline
    // processFrame() will be called to process the frame

    // Remember to change the camera rotation
        phoneCam.startStreaming(width, height, OpenCvCameraRotation.SIDEWAYS_LEFT);


    //...

    ColorDetector.SkystoneLocation location = detector.getLocation();
        if (location != SkystoneDetector.SkystoneLocation.NONE) {
        // Move to the left / right
    } else {
        // Grab the skystone
    }

    // more robot logic...
}