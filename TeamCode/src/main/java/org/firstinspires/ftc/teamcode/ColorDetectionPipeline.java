package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
@Disabled
public class ColorDetectionPipeline extends OpenCvPipeline {

    private Mat hsvMat = new Mat();
    private Mat maskBlue = new Mat();
    private Mat maskYellow = new Mat();
    private Mat hierarchy = new Mat();
    private Mat output = new Mat();

    private Scalar lowerBlue = new Scalar(100, 150, 50); // Lower bound for blue in HSV
    private Scalar upperBlue = new Scalar(140, 255, 255); // Upper bound for blue in HSV

    private Scalar lowerYellow = new Scalar(20, 100, 100); // Lower bound for yellow in HSV
    private Scalar upperYellow = new Scalar(30, 255, 255); // Upper bound for yellow in HSV

    private boolean isBlueDetected = false;
    private boolean isYellowDetected = false;
    private int objectX = -1; // X-coordinate of the detected object
    private int objectY = -1;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Filter for blue and yellow
        Core.inRange(hsvMat, lowerBlue, upperBlue, maskBlue);
        Core.inRange(hsvMat, lowerYellow, upperYellow, maskYellow);

        // Find contours
        List<MatOfPoint> blueContours = new ArrayList<>();
        List<MatOfPoint> yellowContours = new ArrayList<>();
        Imgproc.findContours(maskBlue, blueContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(maskYellow, yellowContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter for the leftmost and straightest object
        isBlueDetected = false;
        isYellowDetected = false;
        objectX = -1;

        MatOfPoint bestContour = null;
        double minX = Double.MAX_VALUE; // Start with a very high x-coordinate
        double bestAspectRatio = 0; // Track the best aspect ratio

        // Process blue contours
        for (MatOfPoint contour : blueContours) {
            double area = Imgproc.contourArea(contour);
            if (area > 100) { // Ignore small objects
                Rect boundingBox = Imgproc.boundingRect(contour);
                double aspectRatio = (double) boundingBox.height / boundingBox.width;

                // Prioritize leftmost object, then straightest
                if (boundingBox.x < minX || (boundingBox.x == minX && aspectRatio > bestAspectRatio)) {
                    minX = boundingBox.x;
                    bestAspectRatio = aspectRatio;
                    bestContour = contour;
                    isBlueDetected = true;
                    isYellowDetected = false;
                }
            }
        }

        // Process yellow contours if no blue object is found
//        for (MatOfPoint contour : yellowContours) {
//            double area = Imgproc.contourArea(contour);
//            if (area > 100) { // Ignore small objects
//                Rect boundingBox = Imgproc.boundingRect(contour);
//                double aspectRatio = (double) boundingBox.height / boundingBox.width;
//
//                // Prioritize leftmost object, then straightest
//                if ((boundingBox.x < minX || (boundingBox.x == minX && aspectRatio > bestAspectRatio)) && !isBlueDetected) {
//                    minX = boundingBox.x;
//                    bestAspectRatio = aspectRatio;
//                    bestContour = contour;
//                    isBlueDetected = false;
//                    isYellowDetected = true;
//                }
//            }
//        }

        // Draw the bounding box around the best object
        input.copyTo(output);
        if (bestContour != null) {
            Rect boundingBox = Imgproc.boundingRect(bestContour);
            Imgproc.rectangle(output, boundingBox.tl(), boundingBox.br(), new Scalar(0, 255, 0), 2);
            objectX = boundingBox.x + (boundingBox.width / 2); // Update objectX
            objectY = boundingBox.y + (boundingBox.height/2);

        }

        return output;
    }

    public boolean isBlueDetected() {
        return isBlueDetected;
    }

    public boolean isYellowDetected() {
        return isYellowDetected;
    }

    public int getObjectX() {
        return objectX;
    }

    public int getObjectY() {
        return objectY;
    }


}
