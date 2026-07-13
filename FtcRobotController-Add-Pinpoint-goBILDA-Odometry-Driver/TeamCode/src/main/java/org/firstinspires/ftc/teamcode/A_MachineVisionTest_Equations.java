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
// imports
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

// This version of the internal camera example uses EasyOpenCV's interface to the original
// Android camera API



@TeleOp(name = "Machine Vision Test Equations", group = "Robot")

public class A_MachineVisionTest_Equations extends LinearOpMode {
   //constants
    DeepRoboConstants robo = new DeepRoboConstants();

    //camera

    OpenCvCamera webcam;
    SampleAlignmentPipeline pipeline;

    private ElapsedTime runtime = new ElapsedTime();

    static Mat returnMat = new Mat();

    static Point samplecenter = new Point();
    static double sampleangle = 0;
    static double grabangle = 0;

    static double sampledist = 0;
    static double servoextend = 0;
    static double disttostrafe = 0;
    static double bottomy = 0;

    static Point testpoint = new Point(320, 240);

    //wrist constants
    double wristtime = 0;
    double wristturn = 0;
    double wristpos = 0;



// auto grab constants
    boolean cangrab = false;
    boolean retract = false;
    double grabtime = 0;
    double retracttime = 0;


    @Override
    public void runOpMode() {

        //hardware map

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        robo.arml = hardwareMap.get(Servo.class, "arml");
        robo.armr = hardwareMap.get(Servo.class, "armr");
        robo.clawrotate = hardwareMap.get(Servo.class, "clawrotate");
        robo.intakewrist = hardwareMap.get(Servo.class, "intakewrist");
        robo.inExtL = hardwareMap.get(Servo.class, "intakeoutl");
        robo.inExtR = hardwareMap.get(Servo.class, "intakeoutr");
        robo.intakepivot = hardwareMap.get(Servo.class, "intakepivot");
        robo.intakeclaw = hardwareMap.get(Servo.class, "intakeclaw");
        robo.pin0 = hardwareMap.digitalChannel.get("color0");
        robo.pin1 = hardwareMap.digitalChannel.get("color1");

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

        //initialization positions

        robo.arml.setPosition(robo.armltransfer);
        robo.armr.setPosition(robo.armrtransfer);
        robo.clawrotate.setPosition(robo.clawrotatescan2);
        robo.intakewrist.setPosition(robo.intakewrist0);
        robo.inExtL.setPosition(robo.intakeleftin);
        robo.inExtR.setPosition(robo.intakerightin);
        robo.intakepivot.setPosition(robo.intakepivotscan);
        robo.intakeclaw.setPosition(robo.intakeopen);


        //wait for start

        telemetry.addLine("Waiting for start");
        telemetry.update();

        // Wait for the user to press start on the Driver Station
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            /*robo.otosloc = robo.otos.getPosition();

            robo.location.x = robo.otosloc.x;
            robo.location.y = robo.otosloc.y;
            robo.location.h = robo.otosloc.h;*/

            // Send some stats to the telemetry
            /*telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());*/
            telemetry.addData("Center: ", pipeline.getCenter().x + ", " + pipeline.getCenter().y);
            telemetry.addData("Angle: ", pipeline.getGrabAngle());
            telemetry.addData("Distance: ", pipeline.getDistance());
            telemetry.addData("Servo Ext: ", pipeline.getServoExt());
            telemetry.addData("Strafe Dist: ", pipeline.getStrafeDist());
            telemetry.update();

            // webcam stops streaming
            if (gamepad1.a) {
                webcam.stopStreaming();
            }

            //region - Test
            /*if (gamepad1.dpad_up) {
                testpoint.y = testpoint.y - 0.005;
            } else if (gamepad1.dpad_down) {
                testpoint.y = testpoint.y + 0.005;
            } else if (gamepad1.dpad_left) {
                testpoint.x = testpoint.x - 0.005;
            } else if (gamepad1.dpad_right) {
                testpoint.x = testpoint.x + 0.005;
            }

            if (testpoint.x < 0) {
                testpoint.x = 0;
            } else if (testpoint.x > 640) {
                testpoint.x = 640;
            }

            if (testpoint.y < 0) {
                testpoint.y = 0;
            } else if (testpoint.y > 480) {
                testpoint.y = 480;
            }*/
            //endregion

            //region - Manual Ext

            //intake out
            if (gamepad1.right_stick_y < -0.75) {
                robo.inExtL.setPosition(robo.inExtL.getPosition() - 0.001);
                robo.inExtR.setPosition(robo.inExtR.getPosition() + 0.001);
            } else if (gamepad1.right_stick_y > 0.75) {
              //intake in
                robo.inExtL.setPosition(robo.inExtL.getPosition() + 0.001);
                robo.inExtR.setPosition(robo.inExtR.getPosition() - 0.001);
            }


            //holding intake in
            if (robo.inExtL.getPosition() > robo.intakeleftin) {
                robo.inExtL.setPosition(robo.intakeleftin);
            } else if (robo.inExtL.getPosition() < robo.intakeleftout) {
                robo.inExtL.setPosition(robo.intakeleftout);
            }


            // flip intake pivot down
            if (robo.inExtL.getPosition() < (robo.intakeleftin - 0.05)) {
                robo.intakepivot.setPosition(robo.intakepivotdown);
            } else {
                robo.intakepivot.setPosition(robo.intakepivotscan);
            }
            //endregion

            //region - Claw
            //claw open/close
            if (gamepad1.left_bumper) {
                robo.intakeclaw.setPosition(robo.intakeclosed);
            } else if (gamepad1.right_bumper) {
                robo.intakeclaw.setPosition(robo.intakeopen);
                //???
                if (robo.inExtL.getPosition() == robo.intakeleftin) {
                    robo.intakepivot.setPosition(robo.intakepivotscan);
                }
            }
            //endregion

            //region - Arm

            //start scan
            if (gamepad1.right_stick_x < -0.75) {
                robo.arml.setPosition(robo.armltransfer);
                robo.armr.setPosition(robo.armrtransfer);
                robo.clawrotate.setPosition(robo.clawrotatescan);
            } else if (gamepad1.right_stick_x > 0.75) {
                //swings arm back
                robo.arml.setPosition(robo.armldeliver);
                robo.armr.setPosition(robo.armrdeliver);
                robo.clawrotate.setPosition(robo.clawrotatedeliverb);
            }
            //endregion

            //region - Wrist
            if ((gamepad1.x || gamepad1.b) && ((runtime.milliseconds() - wristtime) > 100)) {
                wristtime = runtime.milliseconds();
                //adjusting values for intake wrist, setting 1 of 5 different positions
                //hit x once for 45, then again to make it 90
                //hit b once for -45, then again for 90
                if (gamepad1.x) {
                    wristturn = wristturn + 1;
                } else if (gamepad1.b) {
                    wristturn = wristturn - 1;
                }

                if (wristturn < -2) {
                    wristturn = -2;
                } else if (wristturn > 2) {
                    wristturn = 2;
                }

                //taking values set wth buttons and setting a variable to the corresponding servo value
                if (wristturn == -2) {
                    wristpos = robo.intakewrist_90;
                } else if (wristturn == -1) {
                    wristpos = robo.intakewrist_45;
                } else if (wristturn == 0) {
                    wristpos = robo.intakewrist0;
                } else if (wristturn == 1) {
                    wristpos = robo.intakewrist45;
                } else if (wristturn == 2) {
                    wristpos = robo.intakewrist90;
                }

                //setting wrist to the variable set above
                robo.intakewrist.setPosition(wristpos);
            }
            //endregion
//region arm auto test
            //set are to deliver
            if (gamepad2.dpad_up) {

                robo.arml.setPosition(robo.armldeliver);
                robo.armr.setPosition(robo.armrdeliver);
                robo.clawrotate.setPosition(robo.clawrotatedeliverb);

            } else if (gamepad2.dpad_down) {
                //set arm to transfer
                robo.arml.setPosition(robo.armltransfer);
                robo.armr.setPosition(robo.armrtransfer);
                robo.clawrotate.setPosition(robo.clawrotatescan);

            }
            //region - Auto Ext
            if (gamepad1.left_stick_y < (-0.75)) {
                //this is for the intake to automatically grab the samples
                // the pipeline servo value is the amount of intake tics calculated from the camera
                robo.inExtL.setPosition(Range.clip(robo.intakeleftin - pipeline.getServoExt(), robo.intakeleftout, robo.intakeleftin));
                robo.inExtR.setPosition(Range.clip(robo.intakerightin + pipeline.getServoExt(), robo.intakerightin, robo.intakerightout));
                robo.intakepivot.setPosition(robo.intakepivotdown);
                robo.intakeclaw.setPosition(robo.intakeopen);

                //this sets the claw angle to the position found from camera
                if (grabangle == 0) {
                    robo.intakewrist.setPosition(robo.intakewrist0);
                } else if (grabangle == 45) {
                    robo.intakewrist.setPosition(robo.intakewrist45);
                } else if (grabangle == 90) {
                    robo.intakewrist.setPosition(robo.intakewrist90);
                } else if (grabangle == (-45)) {
                    robo.intakewrist.setPosition(robo.intakewrist_45);
                } else if (grabangle == (-90)) {
                    robo.intakewrist.setPosition(robo.intakewrist_90);
                }
                //vestigial code
                cangrab = true;
                //set timer
                grabtime = runtime.milliseconds();
                //retracts intake, retract time time it takes to retract
            } else if ((gamepad1.left_stick_y > 0.75) || (retract && (runtime.milliseconds() > (retracttime + 300)))) {
                robo.inExtL.setPosition(robo.intakeleftin);
                robo.inExtR.setPosition(robo.intakerightin);
                robo.intakepivot.setPosition(robo.intakepivotback);
                robo.intakewrist.setPosition(robo.intakewrist0);
                robo.intakeclaw.setPosition(robo.intakeclosed);

                //vestegial, used to be for color sensor
                cangrab = false;
                // checks if can retract, aready in so set to false
                retract = false;
            }
            //displays color sensor on the intake's value
            if (cangrab && (runtime.milliseconds() > (grabtime + 200))) {
                if (robo.pin0.getState() && robo.pin1.getState()) {
                    robo.colorseen = "Yellow";
                } else if (robo.pin0.getState()) {
                    robo.colorseen = "Blue";
                } else if (robo.pin1.getState()) {
                    robo.colorseen = "Red";
                } else {
                    robo.colorseen = "None";
                }
                //close claw when it sees yellow
                if (robo.colorseen.matches("Yellow")) {
                    robo.intakeclaw.setPosition(robo.intakeclosed);
                    retracttime = runtime.milliseconds();
                    // just grabbed yellow so can retract
                    retract = true;
                    // just grabbed so cannot grab
                    cangrab = false;
                }
            }

            if (gamepad1.dpad_up) {
                // preparing to scan, taking intake in and setting outtake
                robo.intakeclaw.setPosition(robo.intakeopen);
                robo.arml.setPosition(robo.armltransfer);
                robo.armr.setPosition(robo.armrtransfer);
                robo.clawrotate.setPosition(robo.clawrotatescan);
            }
            //endregion

            //region - Cheat Strafe
            /*if (Math.abs(pipeline.getStrafeDist()) > 0) {
                if (pipeline.getStrafeDist() > 2) {
                    robo.fl.setPower(0.5);
                    robo.fr.setPower(-0.5);
                    robo.bl.setPower(-0.5);
                    robo.br.setPower(0.5);
                } else if (pipeline.getStrafeDist() < (-2)) {
                    robo.fl.setPower(-0.5);
                    robo.fr.setPower(0.5);
                    robo.bl.setPower(0.5);
                    robo.br.setPower(-0.5);
                } else if ((pipeline.getStrafeDist() < 1) && (pipeline.getStrafeDist() > 0)) {
                    robo.fl.setPower(0.3);
                    robo.fr.setPower(-0.3);
                    robo.bl.setPower(-0.3);
                    robo.br.setPower(0.3);
                } else if ((pipeline.getStrafeDist() > (-1)) && (pipeline.getStrafeDist() < 0)) {
                    robo.fl.setPower(-0.3);
                    robo.fr.setPower(0.3);
                    robo.bl.setPower(0.3);
                    robo.br.setPower(-0.3);
                } else {
                    DriveStop();
                }
            } else {
                DriveStop();
            }*/
            //endregion
// ask william about camera stuff
            //region - Change Display
            if (gamepad1.left_trigger > 0.5) {
                returnMat = pipeline.morph;
            } else if (gamepad1.right_trigger > 0.5) {
                returnMat = pipeline.dst;
            } else if (gamepad1.y) {
                returnMat = pipeline.result;
            }
            //endregion
        }
    }


//camera code
    static class SampleAlignmentPipeline extends OpenCvPipeline {
        // matrices - ask william or figure out
        // An enum to define the skystone position
        Mat colorG = new Mat();
        Mat dst = new Mat();
        Mat cdst = new Mat();
        Mat hsv = new Mat();
        Mat result = new Mat();

        Mat morph = new Mat();
        Mat lines = new Mat(); // will hold the results of the detection

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile double angles;
// information for drawing green lines on camera display
        Scalar green = new Scalar(120, 255, 180);
        Scalar white = new Scalar(255, 255, 255);

        @Override
        public void init(Mat firstFrame) {
        }

        public void houghPolar(Mat input) {
            //blur image
            Mat blur = new Mat();
            //bilateral filter
            Imgproc.bilateralFilter(input, blur, 3, 59, 117); //81, 136
            //morphology ending, or morphology close then morphology open
            Imgproc.morphologyEx(blur, morph, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                    new Size(7, 7)), new Point(-1, -1), 2);

            // Edge detection
            Imgproc.Canny(morph, dst, 58, 174, 3, false);

            // Copy edges to the images that will display the results in BGR
            Imgproc.cvtColor(dst, cdst, Imgproc.COLOR_GRAY2BGR);
            // Standard Hough Line Transform
            blur.release();
            Imgproc.HoughLinesP(dst, lines, 1, Math.PI / 180, 20, 65, 40); // runs the actual detection
            // Draw the lines

            ArrayList<LineData> AllLines = new ArrayList<>();
            //find the endpoints for the lines,
            for (int x = 0; x < lines.rows(); x++) {
                //set the lines
                double[] l = lines.get(x, 0);
                //find the slope of an edge line drawn
                double slope = (l[3] - l[1]) / (l[2] - l[0]);
                //find the intercept of the same edge line
                double intercept = l[1] - (slope * l[0]);
                //use pythagoras to find lenth of the line from endpoints
                double length = Math.sqrt(Math.pow((l[2] - l[0]), 2) + Math.pow((l[3] - l[1]), 2));
                //draw the line between the endpoints
                LineData linedata = new LineData(new Point(l[0], l[1]), new Point(l[2], l[3]), slope, intercept, length);
                AllLines.add(linedata);
            }

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
                    if (Math.abs((AllLines.get(z).slope - AllLines.get(w).slope) / (AllLines.get(w).slope + 1e-6)) < 0.99) {
                        //checking distance on the points to make sure it can grab it
                        if ((pt1check > 58) && (pt2check > 58) && (pt1check < 110) && (pt2check < 110)) {
                           //draw p lines
                            PLineData parallellines = new PLineData(AllLines.get(z).pt1, AllLines.get(z).pt2,
                                    AllLines.get(w).pt1, AllLines.get(w).pt2);
                            parallel.add(parallellines);
                            //find x value of center
                            double centerx = (Math.min(Math.min(parallellines.pt1.x, parallellines.pt2.x), Math.min(
                                    parallellines.pt3.x, parallellines.pt4.x)) + (Math.max(Math.max(parallellines.pt1.x,
                                    parallellines.pt2.x), Math.max(parallellines.pt3.x, parallellines.pt4.x)))) / 2;
                           // find y value of center
                            double centery = (Math.min(Math.min(parallellines.pt1.y, parallellines.pt2.y), Math.min(
                                    parallellines.pt3.y, parallellines.pt4.y)) + (Math.max(Math.max(parallellines.pt1.y,
                                    parallellines.pt2.y), Math.max(parallellines.pt3.y, parallellines.pt4.y)))) / 2;
                            //set center point with found values
                            Point center = new Point(centerx, centery);
                            //find the height of the sample
                            double h = (Math.sqrt(Math.abs(Math.pow((parallellines.pt3.x - parallellines.pt1.x), 2) +
                                    Math.pow((parallellines.pt3.y - parallellines.pt1.y), 2))) + Math.sqrt(Math.abs(
                                            Math.pow((parallellines.pt4.x - parallellines.pt2.x), 2) +
                                                    Math.pow((parallellines.pt4.y - parallellines.pt2.y), 2)))) / 2;
                           // calculate the area of the sample
                            double area = (0.5) * (AllLines.get(z).length + AllLines.get(w).length) * (h);
                            //find slope of lines that are not line w and z
                            double otherslope1 = (AllLines.get(w).pt1.y - AllLines.get(z).pt1.y) /
                                    (AllLines.get(w).pt1.x - AllLines.get(z).pt1.x);
                            double otherslope2 = (AllLines.get(w).pt2.y - AllLines.get(z).pt2.y) /
                                    (AllLines.get(w).pt2.x - AllLines.get(z).pt2.x);
                            //find angle measures of each corner
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
                            //check if values are in range
                            if ((8 < pixel[0]) && (pixel[0] < 40) && (20 < pixel[1]) && (pixel[1] < 255) && (170 < pixel[2]) && (pixel[2] < 255)) {
                                centeryellow = true;
                            }
                            //check sample is in area and corner angles of sample are not too narrow
                            if ((area > 4750) && (area < 16000) && (((theta1 > 60) && (theta2 > 60)) || ((theta3 > 60) && (theta4 > 60))) && centeryellow) {
                               //draw lines
                                Imgproc.line(dst, parallellines.pt1, parallellines.pt2, green, 3, Imgproc.LINE_AA);
                                Imgproc.line(dst, parallellines.pt3, parallellines.pt4, green, 3, Imgproc.LINE_AA);
                                Imgproc.line(dst, parallellines.pt1, parallellines.pt3, green, 3, Imgproc.LINE_AA);
                                Imgproc.line(dst, parallellines.pt2, parallellines.pt4, green, 3, Imgproc.LINE_AA);
                                //draw centerpoint
                                Imgproc.circle(dst, center, 2, green, 5);

                                samplecenter = center;
                                //set x and y values and angle of the sample
                                double dy = parallellines.pt1.y - parallellines.pt2.y;
                                double dx = parallellines.pt1.x - parallellines.pt2.x;
                                double angle = (Math.atan2(dy, dx) * (180 / Math.PI)) - 90;

                                // set negative values back to positive
                                if (angle < 0) {
                                    angle = angle + 180;
                                }
                                //set found angle as sample angle
                                sampleangle = angle;
                                //set claw rotate positions
                                if (Math.abs(sampleangle) < 30) {
                                    grabangle = 0;
                                } else if (sampleangle >= 30) {
                                    if (sampleangle < 65) {
                                        grabangle = -45;
                                    } else {
                                        grabangle = -90;
                                    }
                                } else if (sampleangle <= (-30)) {
                                    if (sampleangle > (-65)) {
                                        grabangle = 45;
                                    } else {
                                        grabangle = 90;
                                    }
                                }

                                Equations();
                            }
                        }
                    }
                }
            }
            //???
            lines.release();
        }

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV, 4);

            //yellow
            Core.inRange(hsv, new Scalar(8, 20, 170), new Scalar(40, 255, 255), colorG);

            result = input;

            houghPolar(colorG);

            //put found values on stream
            Imgproc.circle(result, samplecenter, 2, green, 5);
            Imgproc.putText(result, ("Center: (" + samplecenter.x + ", " + samplecenter.y + ")"), new Point((samplecenter.x + 20), (samplecenter.y + 18)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
            Imgproc.putText(result, ("Angle: " + Math.round(sampleangle)), new Point((samplecenter.x + 20), (samplecenter.y + 36)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
            Imgproc.putText(result, ("Grab Angle: " + Math.round(grabangle)), new Point((samplecenter.x + 20), (samplecenter.y + 54)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
            Imgproc.putText(result, String.format("Distance: %.2f\n", getDistance()), new Point((samplecenter.x + 20), (samplecenter.y - 36)), //54
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
            /*Imgproc.putText(result, String.format("ServoExt: %.2f\n", getServoExt()), new Point((samplecenter.x + 20), (samplecenter.y - 36)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);*/
            Imgproc.putText(result, String.format("StrafeDist: %.2f\n", getStrafeDist()), new Point((samplecenter.x + 20), (samplecenter.y - 18)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
            /*Imgproc.putText(result, ("Bottom Y: " + Math.round(bottomy)), new Point((samplecenter.x + 20), (samplecenter.y - 72)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);*/

            /*Imgproc.circle(result, testpoint, 2, white, 5);
            Imgproc.putText(result, ("Center: (" + Math.round(testpoint.x) + ", " + Math.round(testpoint.y) + ")"), new Point((testpoint.x -40), (testpoint.y + 15)),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.3, white, 1);*/

            Equations();

            //what to return
            if (!returnMat.empty()) {
                return returnMat;
            } else {
                return result;
            }        }

        public void Equations() {
            //set doubles for equations
            //adjust y level
            double reversey = 480 - samplecenter.y;


            bottomy = reversey;

            double Llim;
            double Rlim;

            double strafedist = 0;
//set limits on what the claw can grab
            if (grabangle == 0) {
                Llim = 275; //270
                Rlim = 365; //390
            } else {
                Llim = 280; //285
                Rlim = 350; //370
            }
//calculations for values
            double distance = ((0.0000000001282) * (Math.pow(reversey, 4))) - ((0.0000001066) * (Math.pow(reversey, 3))) +
                    ((0.00005792) * (Math.pow(reversey, 2))) + (0.0102 * reversey) + 4.75; //2.75//3 //4.5 //0.5 //3.9049

            double servoext = ((-0.00000161) * (Math.pow(distance, 6))) + ((0.00009517) * (Math.pow(distance, 5))) -
                    ((0.002263) * (Math.pow(distance, 4))) + ((0.02786) * (Math.pow(distance, 3))) - ((0.1866) *
                    (Math.pow(distance, 2))) + (0.6626 * distance) - 0.8; //0.75 //0.8491 //0.45

            double pixL = ((-0.0002801) * (Math.pow(distance, 7))) + ((0.0198) *
                    (Math.pow(distance, 6))) - ((0.5866) * (Math.pow(distance, 5))) +
                    ((9.4341) * (Math.pow(distance, 4))) - ((88.7643) * (Math.pow(distance, 3)))
                    + ((487.8807) * (Math.pow(distance, 2))) - (1450.6043 * distance) + 1852.021;
            double pixR = ((-0.0004808) * (Math.pow(distance, 5))) +
                    ((0.02754) * (Math.pow(distance, 4))) - ((0.6007) * (Math.pow(distance, 3)))
                    + ((6.281) * (Math.pow(distance, 2))) - (33.592 * distance) + 124.3077;
//check strafe dis
            if ((samplecenter.x < Llim) || (samplecenter.x > Rlim)) {
                if (samplecenter.x < Llim) {
                    strafedist = (samplecenter.x - 320) / pixL;
                } else {
                    strafedist = (samplecenter.x - 320) / pixR;
                }
            }
//set distances to calculated distance
            sampledist = distance;
            servoextend = servoext;
            disttostrafe = strafedist;
        }
//set public values
        public double getStrafeDist() {
            Equations();
            return disttostrafe;
        }

        public double getServoExt() {
            Equations();
            return servoextend;
        }

        public double getDistance() {
            Equations();
            return sampledist;
        }

        public Point getCenter() {
            Equations();
            return samplecenter;
        }

        public double getGrabAngle() {
            Equations();
            return grabangle;
        }
    }
    //calculate line and plane data

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

