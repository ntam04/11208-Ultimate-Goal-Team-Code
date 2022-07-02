package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamMarkerDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        CENTER,
        RIGHT
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(10, 65),
            new Point(60, 95));
    static final Rect RIGHT_ROI = new Rect(
            new Point(160, 65),
            new Point(210, 95));
    static double PERCENT_COLOR_THRESHOLD = 0.2;

    public TeamMarkerDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 100, 100);
        Scalar highHSV = new Scalar(179, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean tapeLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean tapeRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (tapeRight & !tapeLeft) {
            location = Location.LEFT;
            telemetry.addData("Marker Location", "Left");
        }
        else if (tapeLeft & !tapeRight) {
            location = Location.CENTER;
            telemetry.addData("Marker Location", "Center");
        }
        else if (tapeRight & tapeLeft) {
            location = Location.RIGHT;
            telemetry.addData("Marker Location", "Right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorMarker = new Scalar(255, 0, 0);
        Scalar colorNone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorNone:colorMarker);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.CENTER? colorNone:colorMarker);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}