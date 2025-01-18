package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvWebcam;
@Disabled
public class HSVDetection extends OpenCvPipeline {
    private OpenCvWebcam webcam;
    private Telemetry telemetry;
    private int centerX = -1;
    // Constructor that accepts both webcam and telemetry
    public HSVDetection(OpenCvWebcam webcam, Telemetry telemetry) {
        this.webcam = webcam;
        this.telemetry = telemetry;
    } //
    @Override
    public Mat processFrame(Mat input) {
        // Convert the image to HSV color space
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        // Define HSV ranges for red and yellow detection
        Scalar lowRed = new Scalar(0, 100, 100); // Adjust these values based on your needs
        Scalar highRed = new Scalar(10, 255, 255);
        Scalar lowYellow = new Scalar(20, 100, 100);  // Adjust these values based on your needs
        Scalar highYellow = new Scalar(30, 255, 255);
        // Threshold the HSV image to get red or yellow
        Mat maskRed = new Mat();
        Mat maskYellow = new Mat();
        Core.inRange(hsvMat, lowRed, highRed, maskRed);
        Core.inRange(hsvMat, lowYellow, highYellow, maskYellow);
        // Combine the red and yellow masks
        Mat combinedMask = new Mat();
        Core.addWeighted(maskRed, 1.0, maskYellow, 1.0, 0.0, combinedMask);
        Rect boundingRect = Imgproc.boundingRect(hsvMat); // Get bounding box of the color
        // Get center x-coordinate
        if (boundingRect.width > 0) {
            // Object detected, calculate center X
            centerX = boundingRect.x + (boundingRect.width / 2);
        } else {
            // No object detected, reset centerX to -1
            centerX = -1;
        }
        // Log detection details
        telemetry.addData("Detected Center X", centerX);
        telemetry.update();
        return hsvMat;
    }
    public int getCenterX(){
        return centerX;
    }
}