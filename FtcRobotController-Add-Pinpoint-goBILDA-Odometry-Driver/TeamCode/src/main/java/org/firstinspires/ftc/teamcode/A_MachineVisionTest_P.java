/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Vector;

// This version of the internal camera example uses EasyOpenCV's interface to the original
// Android camera API

@TeleOp(name = "Machine Vision Test P", group = "Robot")

public class A_MachineVisionTest_P extends LinearOpMode {

    //call public constants
    DeepRoboConstants robo = new DeepRoboConstants();
    OpenCvCamera webcam;
    SampleAlignmentPipeline pipeline;

    //name the statics

    static Mat returnMat = new Mat();

    static double slopelim = 0.99; //0.6
    static double distlim = 58; //50
    static double maxdist = 110;

    static double msize = 7;
    static double iterations = 2;

    static double threshold1 = 58; //50

    static double pthreshold = 20;
    static double minlength = 65;
    static double maxgap = 40;

    static double yh = 8;
    static double ys = 20;
    static double yv = 170;
    static double yhm = 40;

    static double minrectarea = 4750;
    static double maxarearect = 16000;
    static double intanglim = 60;

    static Point samplecenter = new Point();
//start
    @Override
    public void runOpMode() {

        //hardware map

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        robo.arml = hardwareMap.get(Servo.class, "arml");
        robo.armr = hardwareMap.get(Servo.class, "armr");
        robo.clawrotate = hardwareMap.get(Servo.class, "clawrotate");

        webcam = OpenCvCameraFactory.getInstance().createWebcam(camera, cameraMonitorViewId);
        pipeline = new SampleAlignmentPipeline();

        // Specify the image processing pipeline we wish to invoke upon receipt of a frame from the camera.
        // Note that switching pipelines on-the-fly (while a streaming session is in flight) *IS* supported.
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //start stream
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });

        robo.arml.setPosition(robo.armltransfer);
        robo.armr.setPosition(robo.armrtransfer);
        robo.clawrotate.setPosition(robo.clawrotatescan);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        // Wait for the user to press start on the Driver Station
        waitForStart();

        while (opModeIsActive()) {
            // Send some stats to the telemetry
            telemetry.addData("MinRectArea: ", minrectarea);
            telemetry.addData("MaxAreaRect: ", maxarearect);
            telemetry.addData("IntAngLim: ", intanglim);
            telemetry.addData("YH: ", yh);
            telemetry.addData("YHM: ", yhm);
            telemetry.addData("YS: ", ys);
            telemetry.addData("YV: ", yv);
            telemetry.addData("PThreshold: ", pthreshold);
            telemetry.addData("Min Length: ", minlength);
            telemetry.addData("Max Gap: ", maxgap);
            telemetry.addData("Threshold1: ", threshold1);
            telemetry.addData("Max Dist: ", maxdist);
            telemetry.addData("Msize: ", msize);
            telemetry.addData("Iterations: ", iterations);
            telemetry.addData("SlopeLim: ", slopelim);
            telemetry.addData("DistLim: ", distlim);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Angle", pipeline.getAnalysis());
            telemetry.update();
//stop stream
            if (gamepad1.a) {
                webcam.stopStreaming();
            }

            //adjust values
            if (gamepad1.dpad_up) {
                distlim = distlim + 0.01;
            } else if (gamepad1.dpad_down) {
                distlim = distlim - 0.01;
            } else if (gamepad1.dpad_right) {
                slopelim = slopelim + 0.0001;
            } else if (gamepad1.dpad_left) {
                slopelim = slopelim - 0.0001;
            } else if (gamepad1.right_bumper) {
                threshold1 = threshold1 + 0.01;
            } else if (gamepad1.left_bumper) {
                threshold1 = threshold1 - 0.01;
            } else if (gamepad1.left_trigger > 0.5) {
                returnMat = pipeline.morph;
            } else if (gamepad1.right_trigger > 0.5) {
                returnMat = pipeline.dst;
            } else if (gamepad1.b) {
                //epsilon = epsilon + 0.001;
                returnMat = pipeline.lines;
            } else if (gamepad1.x) {
                //epsilon = epsilon - 0.001;
                returnMat = pipeline.result;
            } else if (gamepad1.left_stick_y > 0.5) {
                yh = yh + 0.1;
            } else if (gamepad1.left_stick_y < -0.5) {
                yh = yh - 0.1;
            } else if (gamepad1.left_stick_x > 0.5) {
                ys = ys + 0.1;
            } else if (gamepad1.left_stick_x < -0.5) {
                ys = ys - 0.1;
            } else if (gamepad1.right_stick_y > 0.5) {
                yv = yv + 0.1;
            } else if (gamepad1.right_stick_y < -0.5) {
                yv = yv - 0.1;
            } else if (gamepad1.right_stick_x > 0.5) {
                yhm = yhm + 0.1;
            } else if (gamepad1.right_stick_x < -0.5) {
                yhm = yhm - 0.1;
            }
//more adjustments
            if (gamepad2.dpad_up) {
                msize = msize + 0.001;
            } else if (gamepad2.dpad_down) {
                msize = msize - 0.001;
            } else if (gamepad2.dpad_right) {
                iterations = iterations + 0.001;
            } else if (gamepad2.dpad_left) {
                iterations = iterations - 0.001;
            } else if (gamepad2.y) {
                maxdist = maxdist + 0.001;
            } else if (gamepad2.a) {
                maxdist = maxdist - 0.001;
            } else if (gamepad2.b) {
                pthreshold = pthreshold + 0.01;
            } else if (gamepad2.x) {
                pthreshold = pthreshold - 0.01;
            } else if (gamepad2.right_bumper) {
                minlength = minlength + 0.01;
            } else if (gamepad2.left_bumper) {
                minlength = minlength - 0.01;
            } else if (gamepad2.right_trigger > 0.5) {
                maxgap = maxgap + 0.01;
            } else if (gamepad2.left_trigger > 0.5) {
                maxgap = maxgap - 0.01;
            } else if (gamepad2.left_stick_y > 0.5) {
                minrectarea = minrectarea + 1;
            } else if (gamepad2.left_stick_y < -0.5) {
                minrectarea = minrectarea - 1;
            } else if (gamepad2.left_stick_x > 0.5) {
                intanglim = intanglim + 0.01;
            } else if (gamepad2.left_stick_x < -0.5) {
                intanglim = intanglim - 0.01;
            } else if (gamepad2.right_stick_y > 0.5) {
                maxarearect = maxarearect + 1;
            } else if (gamepad2.right_stick_y < -0.5) {
                maxarearect = maxarearect - 1;
            }
        }
    }

    static class SampleAlignmentPipeline extends OpenCvPipeline {
        // An enum to define the skystone position
        Mat colorR0 = new Mat();
        Mat colorR1 = new Mat();
        Mat colorR = new Mat();
        Mat colorG = new Mat();
        Mat colorB = new Mat();
        Mat dst = new Mat();
        Mat cdst = new Mat();
        Mat hsv = new Mat();
        Mat result = new Mat();

        Mat morph = new Mat();
        Mat lines = new Mat(); // will hold the results of the detection

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile double angles;
        //information for drawing the greenlines that are displayed
        Scalar green = new Scalar(120, 255, 180);

        ArrayList<GLineData> GoodLines = new ArrayList<>();
//  initialize the first frame of the matrix
        @Override
        public void init(Mat firstFrame) {
        }

        public boolean areClose(double i1, double i2, double range) {
            return Math.abs(i1 - i2) <= range;
        }
//create the hough lines
        public void houghPolar(Mat input, Scalar color) {
           //blur the image
            Mat blur = new Mat();
            //apply the bilateral filter
            Imgproc.bilateralFilter(input, blur, 3, 59, 117); //81, 136

            Imgproc.morphologyEx(blur, morph, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                    new Size(msize, msize)), new Point(-1, -1), (int) iterations);

            // Edge detection
            Imgproc.Canny(morph, dst, threshold1, (3 * threshold1), 3, false);

            // Copy edges to the images that will display the results in BGR
            Imgproc.cvtColor(dst, cdst, Imgproc.COLOR_GRAY2BGR);
            // Standard Hough Line Transform
            blur.release();
            Imgproc.HoughLinesP(dst, lines, 1, Math.PI / 180, (int) pthreshold, minlength, maxgap); // runs the actual detection
            // Draw the lines



            ArrayList<LineData> AllLines = new ArrayList<>();
            //find endpoints for the lines
            for (int x = 0; x < lines.rows(); x++) {
               //set the lines
                double[] l = lines.get(x, 0);
                //find the slope of the line drawn
                double slope = (l[3] - l[1]) / (l[2] - l[0]);
                //find the intercept of the line
                double intercept = l[1] - (slope * l[0]);
                //find the length of the line
                double length = Math.sqrt(Math.pow((l[2] - l[0]), 2) + Math.pow((l[3] - l[1]), 2));
                //draw line with data earlier set
                LineData linedata = new LineData(new Point(l[0], l[1]), new Point(l[2], l[3]), slope, intercept, length);
                AllLines.add(linedata);

                /*if (checkframe) {
                    GLineData glinedata = new GLineData(new Point(l[0], l[1]), new Point(l[2], l[3]), slope, intercept, length, 2);
                    GoodLines.add(glinedata);

                    Log.d("Lines", "Good Lines Size0: " + GoodLines.size());

                    checkframe = false;
                } else {
                    boolean linechange = true;

                    for (int m = 0; m < GoodLines.size(); m++) {
                        GLineData Glinedata = new GLineData(GoodLines.get(m).pt1, GoodLines.get(m).pt2,
                                GoodLines.get(m).slope, GoodLines.get(m).intercept, GoodLines.get(m).length,
                                (GoodLines.get(m).count - 1));
                        GoodLines.set(m, Glinedata);
                    }

                    Log.d("Lines", "Good Lines Size: " + GoodLines.size());

                    for (int k = 0; k < GoodLines.size(); k++) {
                        if ((Math.abs(GoodLines.get(k).pt1.x - AllLines.get(x).pt1.x) < complim) &&
                                (Math.abs(GoodLines.get(k).pt1.y - AllLines.get(x).pt1.y) < complim) &&
                                (Math.abs(GoodLines.get(k).pt2.x - AllLines.get(x).pt2.x) < complim) &&
                                (Math.abs(GoodLines.get(k).pt2.y - AllLines.get(x).pt2.y) < complim)) {
                            GLineData gLinedata = new GLineData(AllLines.get(x).pt1, AllLines.get(x).pt2, slope,
                                    intercept, length, (GoodLines.get(k).count + 2));
                            GoodLines.set(k, gLinedata);

                            linechange = false;
                        }
                    }

                    Log.d("Lines", "Good Lines Size1: " + GoodLines.size());

                    if (linechange) {
                        GLineData glinedata = new GLineData(AllLines.get(x).pt1, AllLines.get(x).pt2, slope,
                                intercept, length, 2);
                        GoodLines.add(glinedata);
                    }

                    Log.d("Lines", "Good Lines Size2: " + GoodLines.size());
                }*/

                //houghlines
                /*double rho = lines.get(x, 0)[0],
                        theta = lines.get(x, 0)[1];
                double a = Math.cos(theta), b = Math.sin(theta);
                double x0 = a * rho, y0 = b * rho;
                Point pt1 = new Point(Math.round(x0 + 1000 * (-b)), Math.round(y0 + 1000 * (a)));
                Point pt2 = new Point(Math.round(x0 - 1000 * (-b)), Math.round(y0 - 1000 * (a)));

                Imgproc.line(result, pt1, pt2, color, 3, Imgproc.LINE_AA, 0);

                double dy = pt1.y - pt2.y;
                double dx = pt1.x - pt2.x;

                double rawAngle = Math.atan2(dy, dx) * (180 / Math.PI);
                double angle = 180 - Math.abs(rawAngle);
                angles.add(angle * (rawAngle < 0 ? 1 : -1));
                //angles.add(angle < 0 ? 180 - Math.abs(angle) : angle);*/
            }

            /*for (int t = 0; t < GoodLines.size(); t++) {
                if (GoodLines.get(t).count < 0) {
                    GoodLines.remove(t);
                }
            }

            Log.d("Lines", "Good Lines Size3: " + GoodLines.size());*/

            ArrayList<PLineData> parallel = new ArrayList<>();

            //this section draws 2 parallel lines between the 1st and 2nd set of endpoints, completing the rectangle
            for (int z = 0; z < (AllLines.size() - 1); z++) {
                for (int w = z + 1; w < AllLines.size(); w++) {
                    //draw a line between w and z between the first points of the two lines
                    double pt1check = Math.sqrt(Math.abs(Math.pow((AllLines.get(w).pt1.x - AllLines.get(z).pt1.x), 2) +
                            Math.pow((AllLines.get(w).pt1.y - AllLines.get(z).pt1.y), 2)));
                    //draw a line between w and z between the second points of two lines
                    double pt2check = Math.sqrt(Math.abs(Math.pow((AllLines.get(w).pt2.x - AllLines.get(z).pt2.x), 2) +
                            Math.pow((AllLines.get(w).pt2.y - AllLines.get(z).pt2.y), 2)));
                    //making sure line w and line z are parallel to each other, find the percent error the +1e-6 is to avoid dividing by zero
                    if (Math.abs((AllLines.get(z).slope - AllLines.get(w).slope) / (AllLines.get(w).slope + 1e-6)) < slopelim) {
                  //checking distance on the points to make sure it can grab it
                        if ((pt1check > distlim) && (pt2check > distlim) && (pt1check < maxdist) && (pt2check < maxdist)) { //((Math.abs(AllLines.get(w).intercept - AllLines.get(z).intercept)) / (Math.sqrt(1 + (AllLines.get(w).slope * AllLines.get(z).slope))))
                            //if (GoodLines.get(z).count > counter) {

                            /*double dyl1 = Math.abs(AllLines.get(w).pt2.y - AllLines.get(w).pt1.y);
                            double dxl1 = Math.abs(AllLines.get(w).pt2.x - AllLines.get(w).pt1.x);
                            double dyl2 = Math.abs(AllLines.get(z).pt2.y - AllLines.get(z).pt1.y);
                            double dxl2 = Math.abs(AllLines.get(z).pt2.x - AllLines.get(z).pt1.x);

                            double l1 = Math.sqrt(Math.abs(Math.pow((dxl1), 2) + Math.pow((dyl1), 2)));
                            double l2 = Math.sqrt(Math.abs(Math.pow((dxl2), 2) + Math.pow((dyl2), 2)));
                            double l3 = Math.sqrt(Math.abs(Math.pow((AllLines.get(z).pt1.x - AllLines.get(w).pt1.x), 2)
                                    + Math.pow((AllLines.get(z).pt1.y - AllLines.get(w).pt1.y), 2)));
                            double l4 = Math.sqrt(Math.abs(Math.pow((AllLines.get(z).pt2.x - AllLines.get(w).pt2.x), 2)
                                    + Math.pow((AllLines.get(z).pt2.y - AllLines.get(w).pt2.y), 2)));

                            if (l1 > l2) {
                                if (l3 > l4) {
                                    if (AllLines.get(z).pt1.x < AllLines.get(z).pt2.x) {
                                        AllLines.get(z).pt2.x = AllLines.get(z).pt1.x + dxl1;
                                        AllLines.get(z).pt2.y = AllLines.get(z).pt1.y - dyl1;
                                    } else {
                                        AllLines.get(z).pt1.x = AllLines.get(z).pt2.x + dxl1;
                                        AllLines.get(z).pt1.y = AllLines.get(z).pt2.y - dyl1;
                                    }
                                } else {
                                    if (AllLines.get(z).pt1.x > AllLines.get(z).pt2.x) {
                                        AllLines.get(z).pt2.x = AllLines.get(z).pt1.x - dxl1;
                                        AllLines.get(z).pt2.y = AllLines.get(z).pt1.y + dyl1;
                                    } else {
                                        AllLines.get(z).pt1.x = AllLines.get(z).pt2.x - dxl1;
                                        AllLines.get(z).pt1.y = AllLines.get(z).pt2.y + dyl1;
                                    }
                                }
                            } else {
                                if (l3 > l4) {
                                    if (AllLines.get(w).pt1.x < AllLines.get(w).pt2.x) {
                                        AllLines.get(w).pt2.x = AllLines.get(w).pt1.x + dxl1;
                                        AllLines.get(w).pt2.y = AllLines.get(w).pt1.y - dyl1;
                                    } else {
                                        AllLines.get(w).pt1.x = AllLines.get(w).pt2.x + dxl1;
                                        AllLines.get(w).pt1.y = AllLines.get(w).pt2.y - dyl1;
                                    }
                                } else {
                                    if (AllLines.get(w).pt1.x > AllLines.get(w).pt2.x) {
                                        AllLines.get(w).pt2.x = AllLines.get(w).pt1.x - dxl1;
                                        AllLines.get(w).pt2.y = AllLines.get(w).pt1.y + dyl1;
                                    } else {
                                        AllLines.get(w).pt1.x = AllLines.get(w).pt2.x - dxl1;
                                        AllLines.get(w).pt1.y = AllLines.get(w).pt2.y + dyl1;
                                    }
                                }
                            }*/
                            //draw p lines
                            PLineData parallellines = new PLineData(AllLines.get(z).pt1, AllLines.get(z).pt2,
                                    AllLines.get(w).pt1, AllLines.get(w).pt2);
                            parallel.add(parallellines);
                            // find x value of the center
                            double centerx = (Math.min(Math.min(parallellines.pt1.x, parallellines.pt2.x), Math.min(
                                    parallellines.pt3.x, parallellines.pt4.x)) + (Math.max(Math.max(parallellines.pt1.x,
                                    parallellines.pt2.x), Math.max(parallellines.pt3.x, parallellines.pt4.x)))) / 2;
                           // find y value of the center
                            double centery = (Math.min(Math.min(parallellines.pt1.y, parallellines.pt2.y), Math.min(
                                    parallellines.pt3.y, parallellines.pt4.y)) + (Math.max(Math.max(parallellines.pt1.y,
                                    parallellines.pt2.y), Math.max(parallellines.pt3.y, parallellines.pt4.y)))) / 2;
                            //set center point
                            Point center = new Point(centerx, centery);
                            //find heighth of the sample
                            double h = (Math.sqrt(Math.abs(Math.pow((parallellines.pt3.x - parallellines.pt1.x), 2) +
                                    Math.pow((parallellines.pt3.y - parallellines.pt1.y), 2))) + Math.sqrt(Math.abs(
                                            Math.pow((parallellines.pt4.x - parallellines.pt2.x), 2) +
                                                    Math.pow((parallellines.pt4.y - parallellines.pt2.y), 2)))) / 2;
                            // find area
                            double area = (0.5) * (AllLines.get(z).length + AllLines.get(w).length) * (h);
                            //find slope of the lines that are not line z and w
                            double otherslope1 = (AllLines.get(w).pt1.y - AllLines.get(z).pt1.y) /
                                    (AllLines.get(w).pt1.x - AllLines.get(z).pt1.x);
                            double otherslope2 = (AllLines.get(w).pt2.y - AllLines.get(z).pt2.y) /
                                    (AllLines.get(w).pt2.x - AllLines.get(z).pt2.x);
                            //find the angle measures of each corner
                            double theta1 = (Math.atan(Math.abs((AllLines.get(z).slope - otherslope1) /
                                    (1 + (AllLines.get(z).slope * otherslope1))))) * (180 / Math.PI);
                            double theta2 = (Math.atan(Math.abs((AllLines.get(w).slope - otherslope2) /
                                    (1 + (AllLines.get(w).slope * otherslope2))))) * (180 / Math.PI);
                            double theta3 = (Math.atan(Math.abs((AllLines.get(z).slope - otherslope2) /
                                    (1 + (AllLines.get(z).slope * otherslope1))))) * (180 / Math.PI);
                            double theta4 = (Math.atan(Math.abs((AllLines.get(w).slope - otherslope1) /
                                    (1 + (AllLines.get(w).slope * otherslope2))))) * (180 / Math.PI);
                            //set double pixel to the center value
                            double[] pixel = hsv.get((int) center.y, (int) center.x);
                            boolean centeryellow = false;
                            //check if center is in range
                            if ((yh < pixel[0]) && (pixel[0] < yhm) && (ys < pixel[1]) && (pixel[1] < 255) && (yv < pixel[2]) && (pixel[2] < 255)) {
                                centeryellow = true;
                            }
                            //check that area is within range and that corners are correct angle not too narrow
                            if ((area > minrectarea) && (area < maxarearect) && (((theta1 > intanglim) && (theta2 > intanglim)) || ((theta3 > intanglim) && (theta4 > intanglim))) && centeryellow) {
                              //draw lines on display
                                Imgproc.line(dst, parallellines.pt1, parallellines.pt2, green, 3, Imgproc.LINE_AA);
                                Imgproc.line(dst, parallellines.pt3, parallellines.pt4, green, 3, Imgproc.LINE_AA);
                                Imgproc.line(dst, parallellines.pt1, parallellines.pt3, green, 3, Imgproc.LINE_AA);
                                Imgproc.line(dst, parallellines.pt2, parallellines.pt4, green, 3, Imgproc.LINE_AA);
                                //draw centerpoint on display
                                Imgproc.circle(dst, center, 2, green, 5);

                                samplecenter = center;
                            }
                            //}
                        }
                    }
                }
            }
            //???
            lines.release();

        }

        @Override
        public Mat processFrame(Mat input) {
            //put color values on stream
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV, 4);

            //yellow
            Core.inRange(hsv, new Scalar(yh, ys, yv), new Scalar(yhm, 255, 255), colorG);

            //set result as input matrix
            result = input;

           //yellow?
            houghPolar(colorG, new Scalar(255, 255, 0));
            //draw center point
            Imgproc.circle(result, samplecenter, 2, green, 5);


//if it doesnt see it
            if (!returnMat.empty()) {
                return returnMat;
            } else {
                //if it does see it return the result
                return result;
            }
        }

        // Call this from the OpMode thread to obtain the latest analysis
        public double getAnalysis() {
            return angles;
        }
    }


    //set different line data
    private static class LineData {
        Point pt1, pt2;
        double slope, intercept;
        double length;

        LineData(Point pt1, Point pt2, double slope, double intercept, double length) {
            this.pt1 = pt1;
            this.pt2 = pt2;
            this.slope = slope;
            this.intercept = intercept;
            this.length = length;
        }
    }

    private static class GLineData {
        Point pt1, pt2;
        double slope, intercept;
        double length;
        double count;

        GLineData(Point pt1, Point pt2, double slope, double intercept, double length, double count) {
            this.pt1 = pt1;
            this.pt2 = pt2;
            this.slope = slope;
            this.intercept = intercept;
            this.length = length;
            this.count = count;
        }
    }

    private static class PLineData {
        Point pt1, pt2, pt3, pt4;

        PLineData(Point pt1, Point pt2, Point pt3, Point pt4) {
            this.pt1 = pt1;
            this.pt2 = pt2;
            this.pt3 = pt3;
            this.pt4 = pt4;
        }
    }
}
