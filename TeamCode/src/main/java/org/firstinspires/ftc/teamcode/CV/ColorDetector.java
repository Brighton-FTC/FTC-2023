package org.firstinspires.ftc.teamcode.CV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.openftc.easyopencv.OpenCvPipeline;

// Test log 15/3/23:
//Figure out why robot can only drive straight when it the velocity reads that both motors are going
//in the opposite direction based on their velocities, implying that the robot cannot turn.
//Perhaphs shift to a turning mechanism where only 1 of the robot's wheels are turning
// Test og 19/3/23:
//fixed the robot turning issue, now resolve why the robot decides to turn left and right in a constant loop
//Theory 1, make the robot turn at extremely reduced speeds
public class ColorDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        NOT_FOUND
    }
    private Location location;
    static final Rect LEFT_ROI = new Rect(
            new Point(60,35),
            new Point(120,75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140,35),
            new Point(200,75));
    static double PERCENT_COLOR_THRESHOLD =0.4;
    public ColorDetector(Telemetry t) {
        telemetry = t;
    }
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23,50,70);
        Scalar highHSV = new Scalar(32, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] /LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] /RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left Raw Value:", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right Raw Value:", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left Percent Value:", Math.round(leftValue*100) + "%");
        telemetry.addData("Right Percent Value:", Math.round(rightValue*100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneLeft && stoneRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Location", "NOT FOUND");
        }
        if (stoneLeft) {
            location = Location.RIGHT;
            telemetry.addData("Location", "RIGHT");
        } else {
            location = Location.LEFT;
            telemetry.addData("Location", "LEFT");
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);

        Scalar colorStone = new Scalar(255,0,0);
        Scalar colorSkyStone = new Scalar(0,255,0);

        Imgproc.rectangle(mat, LEFT_ROI, location==Location.LEFT? colorSkyStone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location==Location.RIGHT? colorSkyStone:colorStone);

        return mat;
    }
    @Override
    public void run() {

    }
    public void every_tick() {}
}