package org.firstinspires.ftc.teamcode;

import android.util.Log;
import android.util.Size;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

@Disabled
@Autonomous(name = "Deep_Auto_Champs", group = "Linear Opmode")
public class Deep_Auto_Champs extends LinearOpMode {

    DeepRoboConstants robo = new DeepRoboConstants();

    //region - Variables
    private ElapsedTime runtime = new ElapsedTime();

    double drivetimeout = 0;
    double cyclecount = 1;
    double tradetimelimit = 0;
    double rtolerance = 0;
    double liftupto = 0;
    double grabtime = 0;
    double closetime = 0;
    double scantime = 0;
    double extradrivetime = 0;
    double yellowcount = 0;
    boolean lift = false;
    boolean transfer = false;
    boolean hascolor = false;
    boolean loopcount = false;
    boolean intake = false;
    boolean stopdrive = false;
    boolean sampleadjust = false;
    boolean park = false;
    boolean clawgrab = false;
    boolean resetotos = false;
    boolean resetready = false;
    boolean adjust = false;
    boolean grabbing = false;
    boolean closemark = false;
    boolean afterfix = false;
    boolean hassample = false;
    boolean armmove = false;

    SparkFunOTOS.Pose2D rightcycledrive1 = new SparkFunOTOS.Pose2D(0, 0, 0);
    SparkFunOTOS.Pose2D rightcycledrive2 = new SparkFunOTOS.Pose2D(0, 0, 0);
    SparkFunOTOS.Pose2D rightdrive1 = new SparkFunOTOS.Pose2D(0, 0, 0);
    SparkFunOTOS.Pose2D righttradedrive = new SparkFunOTOS.Pose2D(0, 0, 0);

    SparkFunOTOS.Pose2D lastpos = new SparkFunOTOS.Pose2D(0, 0, 0);
    //endregion

    @Override
    public void runOpMode() {
        //region - Hardware Map
        robo.fl = hardwareMap.get(DcMotor.class, "fl");
        robo.fr = hardwareMap.get(DcMotor.class, "fr");
        robo.bl = hardwareMap.get(DcMotor.class, "bl");
        robo.br = hardwareMap.get(DcMotor.class, "br");

        robo.lift1 = hardwareMap.get(DcMotor.class, "lift1");
        robo.lift2 = hardwareMap.get(DcMotor.class, "lift2");
        robo.lift3 = hardwareMap.get(DcMotor.class, "lift3");

        robo.arml = hardwareMap.get(Servo.class, "arml");
        robo.armr = hardwareMap.get(Servo.class, "armr");

        robo.inExtL = hardwareMap.get(Servo.class, "intakeoutl");
        robo.inExtR = hardwareMap.get(Servo.class, "intakeoutr");

        robo.intakeclaw = hardwareMap.get(Servo.class, "intakeclaw");
        robo.claw = hardwareMap.get(Servo.class, "claw");
        robo.clawrotate = hardwareMap.get(Servo.class, "clawrotate");
        robo.intakepivot = hardwareMap.get(Servo.class, "intakepivot");
        robo.intakewrist = hardwareMap.get(Servo.class, "intakewrist");

        robo.Bus = hardwareMap.get(AnalogInput.class, "ultrab");
        robo.Rus = hardwareMap.get(AnalogInput.class, "ultrar");
        robo.Lus = hardwareMap.get(AnalogInput.class, "ultral");

        robo.liftMag = hardwareMap.get(DigitalChannel.class, "lift_mag");
        robo.clawbeam = hardwareMap.get(DigitalChannel.class, "clawbeam");

        robo.pin0 = hardwareMap.digitalChannel.get("color0");
        robo.pin1 = hardwareMap.digitalChannel.get("color1");

        robo.otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        //robo.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        robo.fl.setDirection(DcMotor.Direction.REVERSE);
        robo.bl.setDirection(DcMotor.Direction.REVERSE);

        robo.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robo.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robo.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robo.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robo.lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        robo.lift3.setDirection(DcMotorSimple.Direction.REVERSE);

        robo.lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.lift3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robo.lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robo.lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robo.fl.setPower(0);
        robo.fr.setPower(0);
        robo.bl.setPower(0);
        robo.br.setPower(0);

        robo.lift1.setPower(0);
        robo.lift2.setPower(0);
        robo.lift3.setPower(0);

        robo.inExtL.setPosition(robo.intakeleftin);
        robo.inExtR.setPosition(robo.intakerightin);

        robo.claw.setPosition(robo.clawclosed);
        robo.intakeclaw.setPosition(robo.intakeopen);
        robo.intakewrist.setPosition(robo.intakewrist0);

        //robo.configureOdo();
        robo.configureOtos();
        //endregion

        //region - Menu
        while ((!gamepad1.a) && opModeInInit()) {
            if (gamepad1.dpad_left) {
                robo.startpos = "Left";
            } else if (gamepad1.dpad_right) {
                robo.startpos = "Right";
            }

            if (gamepad1.y) {
                robo.colortarget = ColorRange.YELLOW;
                robo.GrabColor = "Yellow";
            } else if (gamepad1.b) {
                robo.colortarget = ColorRange.RED;
                robo.teamcolor = "Red";
                robo.GrabColor = "Red";
            } else if (gamepad1.x) {
                robo.colortarget = ColorRange.BLUE;
                robo.teamcolor = "Blue";
                robo.GrabColor = "Blue";
            }

            telemetry.addData("Press Left for Left", "");
            telemetry.addData("Press Right for Right", "");
            telemetry.addData("Press Colors for Cycle", "");
            telemetry.addData("Press A to Continue", "");
            telemetry.addData("Start Position: ", robo.startpos);
            telemetry.addData("Color to Grab: ", robo.GrabColor);
            telemetry.addData("Team Color Is: ", robo.teamcolor);
            telemetry.update();
        }

        /*
        if (robo.startpos.matches("Right")) {
            while (!gamepad1.dpad_down) {
                if (gamepad1.dpad_left) {
                    robo.grabloc = "Submersible";
                    robo.barfront = true;
                } else if (gamepad1.dpad_right) {
                    robo.grabloc = "Barcode";
                    robo.barfront = false;
                }

                telemetry.addData("Press Left for Barcode", "");
                telemetry.addData("Press Right for Submersible", "");
                telemetry.addData("Press Down to Continue", "");
                telemetry.addData("Grab Location: ", robo.grabloc);
                telemetry.update();
            }
        }*/
        //endregion

        //region - Camera Init
        /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
         * - Specify the color range you are looking for.  You can use a predefined color, or create you own color range
         *     .setTargetColorRange(ColorRange.BLUE)                      // use a predefined color match
         *       Available predefined colors are: RED, BLUE YELLOW GREEN
         *     .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,      // or define your own color match
         *                                           new Scalar( 32, 176,  0),
         *                                           new Scalar(255, 255, 132)))
         *
         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *         ImageRegion.entireFrame()
         *         ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixel square near the upper left corner
         *         ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height square centered on screen
         *
         * - Define which contours are included.
         *     You can get ALL the contours, or you can skip any contours that are completely inside another contour.
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)  // return all contours
         *        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)            // exclude contours inside other contours
         *        note: EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up areas of solid color.
         *
         * - turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
         *        .setDrawContours(true)
         *
         * - include any pre-processing of the image or mask before looking for Blobs.
         *     There are some extra processing you can include to improve the formation of blobs.  Using these features requires
         *     an understanding of how they may effect the final blobs.  The "pixels" argument sets the NxN kernel size.
         *        .setBlurSize(int pixels)    Blurring an image helps to provide a smooth color transition between objects, and smoother contours.
         *                                    The higher the number of pixels, the more blurred the image becomes.
         *                                    Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *                                    Blurring too much may hide smaller features.  A "pixels" size of 5 is good for a 320x240 image.
         *        .setErodeSize(int pixels)   Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *                                    Erosion can grow holes inside regions, and also shrink objects.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         *        .setDilateSize(int pixels)  Dilation makes objects more visible by filling in small holes, making lines appear thicker,
         *                                    and making filled shapes appear larger. Dilation is useful for joining broken parts of an
         *                                    object, such as when removing noise from an image.
         *                                    "pixels" in the range of 2-4 are suitable for low res images.
         */
        /*robo.colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(robo.colortarget)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(0)                               // Smooth the transitions between different colors in image
                .build();*/

        /*
         * Build a vision portal to run the Color Locator process.
         *
         *  - Add the colorLocator process created above.
         *  - Set the desired video resolution.
         *      Since a high resolution will not improve this process, choose a lower resolution that is
         *      supported by your camera.  This will improve overall performance and reduce latency.
         *  - Choose your video source.  This may be
         *      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  .....   for a webcam
         *  or
         *      .setCamera(BuiltinCameraDirection.BACK)    ... for a Phone Camera
         */
        /*robo.portal = new VisionPortal.Builder()
                .addProcessor(robo.colorLocator)
                .setCameraResolution(new Size(robo.w, robo.h))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();*/
        //endregion

        //region - Diff Claw Pos for L and R
        /*if (robo.startpos.matches("Left")) {
            robo.arml.setPosition(robo.armltransfer);
            robo.armr.setPosition(robo.armrtransfer);
            robo.clawrotate.setPosition(robo.clawrotateup);
            robo.intakepivot.setPosition(robo.intakepivotup);
        } else {*/
        robo.clawrotate.setPosition(robo.clawrotateinit);
        robo.arml.setPosition(robo.armlinit);
        robo.armr.setPosition(robo.armrinit);
        robo.intakepivot.setPosition(robo.intakepivotback);
        //}
        //endregion

        //region - Match Prep
        navreset(true, true, true, true);

        robo.roboorigin = new SparkFunOTOS.Pose2D(0, 0, 0);

        robo.otos.setPosition(robo.roboorigin);
        //robo.odo.setPosition(robo.odoorigin);
        //robo.odo.update();

        telemetry.addData("Start Position: ", robo.startpos);
        telemetry.addData("Otos x: ", robo.otos.getPosition().x);
        telemetry.addData("Otos y: ", robo.otos.getPosition().y);
        telemetry.addData("Otos z: ", robo.otos.getPosition().h);
        /*telemetry.addData("Odo x: ", robo.odo.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Odo y: ", robo.odo.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("Odo z: ", robo.odo.getPosition().getHeading(AngleUnit.DEGREES));*/
        telemetry.addData("Status: ", "Ready");
        telemetry.update();
        //endregion

        waitForStart();
        runtime.reset();

        //region - Location SetUp
        //robo.odopos = robo.odo.getPosition();
        robo.otosloc = robo.otos.getPosition();

        robo.location.x = robo.otosloc.x;
        robo.location.y = robo.otosloc.y;
        robo.location.h = robo.otosloc.h;
        //endregion

        if (robo.startpos.matches("Left")) {
            ServoArm(robo.armlup, robo.armrup);
            ClawRot(robo.clawrotatedeliverb);

            drivetimeout = runtime.milliseconds() + 1300; //1600
            resetready = true;
            while (((robo.lift1.getCurrentPosition() < robo.lifthigh) || (runtime.milliseconds() < (drivetimeout)))
                    && opModeIsActive()) { //below was -300 ms
                if ((runtime.milliseconds() < (drivetimeout)) && (((Math.abs(robo.deliver1.x - robo.location.x) >
                        0.25)) || (Math.abs(robo.deliver1.y - robo.location.y) > 0.25) || (Math.abs(robo.deliver1.h -
                        robo.location.h) > 0.25))) {
                    XYZ(robo.deliver1, 8, true, false, 0.3); //XYZ?
                } else if (resetready) {
                    DriveHold(false);
                    DriveStop();
                    resetotos = true;
                    resetready = false;
                }

                /*if (resetotos && ((Math.abs(robo.otos.getVelocity().x + robo.otos.getVelocity().y + robo.otos.getVelocity().h)) < 0.5)) {
                    navreset(true, true, true, true);
                    resetotos = false;
                }*/

                LiftMulti(robo.lifthigh, runtime.milliseconds());
            }

            DriveHold(false);
            DriveStop();
            LiftStop(robo.lifthigh);
            Pos(robo.deliver1);

            DeliverLeft(false);

            //region - Cycles
            LeftBarcodeGrab(robo.floorgrabR, robo.intakewrist0, false);

            DeliverLeft(false);

            LeftBarcodeGrab(robo.floorgrabM, robo.intakewrist0, false);

            DeliverLeft(false);

            LeftBarcodeGrab(robo.floorgrabL, robo.intakewrist_45, true);

            DeliverLeft(false);
            //endregion

            LSubGrab(3);

            if (hassample) {
                DeliverLeft(true);
                hassample = false;
            }

            if (runtime.milliseconds() < 23000) {
                LSubGrab(2);

                if ((!park) && hassample) {
                    DeliverLeft(true);
                    hassample = false;
                }
            }

            if (!park) {
                ParkLeft();
            }
        } else {
            if (robo.teamcolor.matches("Red")) {
                robo.colortodrop = "Blue";
            } else {
                robo.colortodrop = "Red";
            }

            DeliverRightBar(1, robo.colortodrop);

            ParkRight();

            sleep(500);
        }
    }

    //region - Drive
    public void navreset(boolean reset_OTOS, boolean recal_OTOS_IMU, boolean reset_opods, boolean recal_opods) {
        if (reset_OTOS) {
            robo.otos.resetTracking();
        }

        if (recal_OTOS_IMU) {
            robo.otos.calibrateImu();
        }

        if (reset_opods) {
            //robo.odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        }

        if (recal_opods) {
            //robo.odo.recalibrateIMU(); //recalibrates the IMU without resetting position
        }
    }

    public void XYZ(SparkFunOTOS.Pose2D driveto, double tolerance, boolean otos, boolean odo, double turnmaxpower) {
        //robo.odo.update();

        Log.d("PositionCheck", "Started Drive Method");

        robo.drivetarget = driveto;

        //region - Logs
        Log.d("PositionCheck", "Target = " + driveto.x + ", " + driveto.y + ", " + driveto.h);
        Log.d("PositionCheck", "YTarget: " + robo.drivetarget.y);
        Log.d("PositionCheck", "XTarget: " + robo.drivetarget.x);
        Log.d("PositionCheck", "ZTarget: " + robo.drivetarget.h);
        //endregion

        robo.ytarget = robo.drivetarget.y;
        robo.xtarget = robo.drivetarget.x;
        robo.ztarget = robo.drivetarget.h;

        //robo.odopos = robo.odo.getPosition();
        robo.otosloc = robo.otos.getPosition();
        robo.roboorigin = new SparkFunOTOS.Pose2D(0, 0, 0);

        //region - Pos by AVG
        /*if (otos && odo) {
            robo.location.x = (robo.otosloc.x + robo.odopos.getX(DistanceUnit.INCH)) / 2;
            robo.location.y = (robo.otosloc.y + robo.odopos.getY(DistanceUnit.INCH)) / 2;
            robo.location.h = (robo.otosloc.h + robo.odopos.getHeading(AngleUnit.DEGREES)) / 2;
        } else if (odo) {
            robo.location.x = robo.odopos.getX(DistanceUnit.INCH);
            robo.location.y = robo.odopos.getY(DistanceUnit.INCH);
            robo.location.h = robo.odopos.getHeading(AngleUnit.DEGREES);
        } else if (otos) {*/
        robo.location.x = robo.otosloc.x;
        robo.location.y = robo.otosloc.y;
        robo.location.h = robo.otosloc.h;
        //}

        Log.d("PositionCheck", "X is: " + robo.location.x);
        Log.d("PositionCheck", "Y is: " + robo.location.y);
        Log.d("PositionCheck", "Z is: " + robo.location.h);

        Log.d("PositionCheck", "Otos X is: " + robo.otosloc.x);
        Log.d("PositionCheck", "Otos Y is: " + robo.otosloc.y);
        Log.d("PositionCheck", "Otos Z is: " + robo.otosloc.h);

        /*Log.d("PositionCheck", "Odo X is: " + robo.odopos.getX(DistanceUnit.INCH));
        Log.d("PositionCheck", "Odo Y is: " + robo.odopos.getY(DistanceUnit.INCH));
        Log.d("PositionCheck", "Odo Z is: " + robo.odopos.getHeading(AngleUnit.DEGREES));

/*
        //region - Error Catch
        if ((Math.abs(robo.otosloc.x - robo.odopos.getX(DistanceUnit.INCH))) > 1) {
            Log.d("Calculations", "Drive Error: X Opod = " + robo.odopos.getX(DistanceUnit.INCH));
            Log.d("Calculations", "Drive Error: X OTOS = " + robo.otosloc.x);
        }

        if ((Math.abs(robo.otosloc.y - robo.odopos.getY(DistanceUnit.INCH))) > 1) {
            Log.d("Calculations", "Drive Error: Y Opod = " + robo.odopos.getY(DistanceUnit.INCH));
            Log.d("Calculations", "Drive Error: Y OTOS = " + robo.otosloc.y);
        }

        if ((Math.abs(robo.otosloc.h - robo.odopos.getHeading(AngleUnit.DEGREES))) > 3) {
            Log.d("Calculations", "Drive Error: H Opod = " + robo.odopos.getHeading(AngleUnit.DEGREES));
            Log.d("Calculations", "Drive Error: H OTOS = " + robo.otosloc.h);
        }
        //endregion*/
        //endregion

        if (((Math.abs(robo.ytarget - robo.location.y)) > 0.25) || ((Math.abs(robo.xtarget - robo.location.x)) > 0.25) ||
                ((Math.abs(robo.ztarget - robo.location.h)) > 1)) {
            Log.d("PositionCheck", "In Drive Method Loop");

            //region - OTOS Drive
            //region - Universal Tracking
            if ((Math.abs(robo.ytarget - robo.location.y)) > tolerance) { //15
                if ((robo.ytarget - robo.location.y) > 0) {
                    robo.yerror = 1;
                } else if ((robo.ytarget - robo.location.y) < 0) {
                    robo.yerror = -1;
                }
            } else {
                robo.yerror = Range.clip(((robo.ytarget - robo.location.y) / tolerance), -1, 1); //15
            }

            if ((Math.abs(robo.xtarget - robo.location.x)) > tolerance) { //15
                if ((robo.xtarget - robo.location.x) > 0) {
                    robo.xerror = 1;
                } else if ((robo.xtarget - robo.location.x) < 0) {
                    robo.xerror = -1;
                }
            } else {
                robo.xerror = Range.clip(((robo.xtarget - robo.location.x) / tolerance), -1, 1); //15
            }

            if (robo.ztarget == robo.drive180.h) {
                if (robo.location.h > 0) {
                    robo.headingdiff = Math.abs(robo.ztarget - robo.location.h);
                } else if (robo.location.h < 0) {
                    robo.headingdiff = Math.abs(robo.ztarget + robo.location.h);
                }
            } else {
                robo.headingdiff = Math.abs(robo.ztarget - robo.location.h);
            }

            if (robo.headingdiff > 25) {
                if (!(robo.ztarget == robo.drive180.h)) {
                    if ((robo.ztarget - robo.location.h) > 0) {
                        robo.zerror = turnmaxpower;
                    } else if ((robo.ztarget - robo.location.h) < 0) {
                        robo.zerror = -turnmaxpower;
                    }
                } else {
                    if (robo.location.h > 0) {
                        robo.zerror = turnmaxpower;
                    } else if (robo.location.h < 0) {
                        robo.zerror = -turnmaxpower;
                    }
                }
            } else {
                if (!(robo.ztarget == robo.drive180.h)) {
                    robo.zerror = Range.clip(((robo.ztarget - robo.location.h) / 25), -0.2, 0.2);
                } else {
                    if (robo.location.h > 0) {
                        robo.zerror = Range.clip(((robo.ztarget - robo.location.h) / 25), -0.2, 0.2);
                    } else if (robo.location.h < 0) {
                        robo.zerror = Range.clip((-(robo.ztarget + robo.location.h) / 25), -0.2, 0.2);
                    }
                }
            }

            Log.d("PositionCheck", "X error is: " + robo.xerror);
            Log.d("PositionCheck", "Y error is: " + robo.yerror);
            Log.d("PositionCheck", "Z error is: " + robo.zerror);

            robo.ypower = robo.yerror;
            robo.xpower = robo.xerror;
            robo.zpower = robo.zerror;

            if (Math.abs(robo.yerror) < robo.y_min) {
                if (Math.abs(robo.yerror) > robo.y_tolerance) {
                    if (robo.yerror > 0) {
                        robo.ypower = robo.y_min;
                    } else if (robo.yerror < 0) {
                        robo.ypower = -robo.y_min;
                    }
                } else {
                    robo.ypower = 0;
                }
            }

            if (Math.abs(robo.xerror) < robo.x_min) {
                if (Math.abs(robo.xerror) > robo.x_tolerance) {
                    if (robo.xerror > 0) {
                        robo.xpower = robo.x_min;
                    } else if (robo.xerror < 0) {
                        robo.xpower = -robo.x_min;
                    }
                } else {
                    robo.xpower = 0;
                }
            }

            if (Math.abs(robo.zerror) < robo.z_min) {
                if (Math.abs(robo.zerror) > robo.z_tolerance) {
                    if (robo.zerror > 0) {
                        robo.zpower = robo.z_min;
                    } else if (robo.zerror < 0) {
                        robo.zpower = -robo.z_min;
                    }
                } else {
                    robo.zpower = 0;
                }
            }
            //endregion

            //flip x and y powers when cross 90 threshold to correct
            if (Math.abs(robo.location.h) > 90) {
                robo.ypower = -robo.ypower;
                robo.xpower = -robo.xpower;
            }

            if ((robo.ztarget == robo.drive180.h) && (robo.headingdiff < 2)) {
                robo.zpower = 0;
            }

            robo.xp = robo.xpower;
            robo.yp = robo.ypower;

            if ((-120 < robo.location.h) && (robo.location.h < -60)) {
                robo.xpower = -robo.yp;
                robo.ypower = robo.xp;
            } else if ((120 > robo.location.h) && (robo.location.h > 60)) {
                robo.xpower = robo.yp;
                robo.ypower = -robo.xp;
            }

            Log.d("PositionCheck", "X power is: " + robo.xpower);
            Log.d("PositionCheck", "Y power is: " + robo.ypower);
            Log.d("PositionCheck", "Z power is: " + robo.zpower);

            robo.flpower = Range.clip(robo.ypower + robo.xpower - robo.zpower, -1, 1);
            robo.frpower = Range.clip(robo.ypower - robo.xpower + robo.zpower, -1, 1);
            robo.blpower = Range.clip(robo.ypower - robo.xpower - robo.zpower, -1, 1);
            robo.brpower = Range.clip(robo.ypower + robo.xpower + robo.zpower, -1, 1);

            //region - OTOS Logs
            Log.d("Test", "Y Value: " + robo.location.y);
            Log.d("Test", "X Value: " + robo.location.x);
            Log.d("Test", "Z Value: " + robo.location.h);
            //endregion
            //endregion

            //region - Apply Motor Power
            //if (drivetime.milliseconds() < timelimit) {
            robo.fl.setPower(robo.flpower);
            robo.fr.setPower(robo.frpower);
            robo.bl.setPower(robo.blpower);
            robo.br.setPower(robo.brpower);

            Log.d("PositionCheck", "FR power is: " + robo.frpower);
            Log.d("PositionCheck", "FL power is: " + robo.flpower);
            Log.d("PositionCheck", "BR power is: " + robo.brpower);
            Log.d("PositionCheck", "BL power is: " + robo.blpower);
            //endregion
        }
    }

    public void FAST(SparkFunOTOS.Pose2D driveto, boolean otos, boolean odo, double turnmaxpower) {
        //robo.odo.update();

        Log.d("PositionCheck", "Started Drive Method");

        robo.drivetarget = driveto;

        //region - Logs
        Log.d("PositionCheck", "Target = " + driveto.x + ", " + driveto.y + ", " + driveto.h);
        /*Log.d("PositionCheck", "YTarget: " + robo.drivetarget.y);
        Log.d("PositionCheck", "XTarget: " + robo.drivetarget.x);
        Log.d("PositionCheck", "ZTarget: " + robo.drivetarget.h);*/
        //endregion

        robo.ytarget = robo.drivetarget.y;
        robo.xtarget = robo.drivetarget.x;
        robo.ztarget = robo.drivetarget.h;

        //robo.odopos = robo.odo.getPosition();
        robo.otosloc = robo.otos.getPosition();
        robo.roboorigin = new SparkFunOTOS.Pose2D(0, 0, 0);

        //region - Pos by AVG
        /*if (otos && odo) {
            robo.location.x = (robo.otosloc.x + robo.odopos.getX(DistanceUnit.INCH)) / 2;
            robo.location.y = (robo.otosloc.y + robo.odopos.getY(DistanceUnit.INCH)) / 2;
            robo.location.h = (robo.otosloc.h + robo.odopos.getHeading(AngleUnit.DEGREES)) / 2;
        } else if (odo) {
            robo.location.x = robo.odopos.getX(DistanceUnit.INCH);
            robo.location.y = robo.odopos.getY(DistanceUnit.INCH);
            robo.location.h = robo.odopos.getHeading(AngleUnit.DEGREES);
        } else if (otos) {*/
        robo.location.x = robo.otosloc.x;
        robo.location.y = robo.otosloc.y;
        robo.location.h = robo.otosloc.h;
        //}

        Log.d("PositionCheck", "X is: " + robo.location.x);
        Log.d("PositionCheck", "Y is: " + robo.location.y);
        Log.d("PositionCheck", "Z is: " + robo.location.h);

        Log.d("PositionCheck", "Otos X is: " + robo.otosloc.x);
        Log.d("PositionCheck", "Otos Y is: " + robo.otosloc.y);
        Log.d("PositionCheck", "Otos Z is: " + robo.otosloc.h);

        /*Log.d("PositionCheck", "Odo X is: " + robo.odopos.getX(DistanceUnit.INCH));
        Log.d("PositionCheck", "Odo Y is: " + robo.odopos.getY(DistanceUnit.INCH));
        Log.d("PositionCheck", "Odo Z is: " + robo.odopos.getHeading(AngleUnit.DEGREES));

/*
        //region - Error Catch
        if ((Math.abs(robo.otosloc.x - robo.odopos.getX(DistanceUnit.INCH))) > 1) {
            Log.d("Calculations", "Drive Error: X Opod = " + robo.odopos.getX(DistanceUnit.INCH));
            Log.d("Calculations", "Drive Error: X OTOS = " + robo.otosloc.x);
        }

        if ((Math.abs(robo.otosloc.y - robo.odopos.getY(DistanceUnit.INCH))) > 1) {
            Log.d("Calculations", "Drive Error: Y Opod = " + robo.odopos.getY(DistanceUnit.INCH));
            Log.d("Calculations", "Drive Error: Y OTOS = " + robo.otosloc.y);
        }

        if ((Math.abs(robo.otosloc.h - robo.odopos.getHeading(AngleUnit.DEGREES))) > 3) {
            Log.d("Calculations", "Drive Error: H Opod = " + robo.odopos.getHeading(AngleUnit.DEGREES));
            Log.d("Calculations", "Drive Error: H OTOS = " + robo.otosloc.h);
        }
        //endregion*/
        //endregion

        if (((Math.abs(robo.ytarget - robo.location.y)) > 0.25) || ((Math.abs(robo.xtarget - robo.location.x)) > 0.25) ||
                ((Math.abs(robo.ztarget - robo.location.h)) > 1)) {
            Log.d("PositionCheck", "In Drive Method Loop");

            //region - OTOS Drive
            //region - Universal Tracking
            if ((Math.abs(robo.ytarget - robo.location.y)) > 3) {
                if ((robo.ytarget - robo.location.y) > 0) {
                    robo.yerror = 1;
                } else if ((robo.ytarget - robo.location.y) < 0) {
                    robo.yerror = -1;
                }

                robo.fastloopy = true;
            } else if (robo.fastloopy) {
                if ((robo.ytarget - robo.location.y) > 0) {
                    robo.yerror = -0.1;
                } else if ((robo.ytarget - robo.location.y) < 0) {
                    robo.yerror = 0.1;
                }

                robo.fastloopy = false;
            } else {
                robo.yerror = Range.clip(((robo.ytarget - robo.location.y) / 10), -0.3, 0.3);
            }

            if ((Math.abs(robo.xtarget - robo.location.x)) > 3) {
                if ((robo.xtarget - robo.location.x) > 0) {
                    robo.xerror = 1;
                } else if ((robo.xtarget - robo.location.x) < 0) {
                    robo.xerror = -1;
                }

                robo.fastloopx = true;
            } else if (robo.fastloopx) {
                if ((robo.xtarget - robo.location.x) > 0) {
                    robo.xerror = -0.1;
                } else if ((robo.xtarget - robo.location.x) < 0) {
                    robo.xerror = 0.1;
                }

                robo.fastloopx = false;
            } else {
                robo.xerror = Range.clip(((robo.xtarget - robo.location.x) / 10), -0.3, 0.3);
            }

            if (robo.ztarget == robo.drive180.h) {
                if (robo.location.h > 0) {
                    robo.headingdiff = Math.abs(robo.ztarget - robo.location.h);
                } else if (robo.location.h < 0) {
                    robo.headingdiff = Math.abs(robo.ztarget + robo.location.h);
                }
            } else {
                robo.headingdiff = Math.abs(robo.ztarget - robo.location.h);
            }

            if (robo.headingdiff > 25) {
                if (!(robo.ztarget == robo.drive180.h)) {
                    if ((robo.ztarget - robo.location.h) > 0) {
                        robo.zerror = turnmaxpower;
                    } else if ((robo.ztarget - robo.location.h) < 0) {
                        robo.zerror = -turnmaxpower;
                    }
                } else {
                    if (robo.location.h > 0) {
                        robo.zerror = turnmaxpower;
                    } else if (robo.location.h < 0) {
                        robo.zerror = -turnmaxpower;
                    }
                }
            } else {
                if (!(robo.ztarget == robo.drive180.h)) {
                    robo.zerror = Range.clip(((robo.ztarget - robo.location.h) / 25), -0.2, 0.2);
                } else {
                    if (robo.location.h > 0) {
                        robo.zerror = Range.clip(((robo.ztarget - robo.location.h) / 25), -0.2, 0.2);
                    } else if (robo.location.h < 0) {
                        robo.zerror = Range.clip((-(robo.ztarget + robo.location.h) / 25), -0.2, 0.2);
                    }
                }
            }

            Log.d("PositionCheck", "X error is: " + robo.xerror);
            Log.d("PositionCheck", "Y error is: " + robo.yerror);
            Log.d("PositionCheck", "Z error is: " + robo.zerror);

            robo.ypower = robo.yerror;
            robo.xpower = robo.xerror;
            robo.zpower = robo.zerror;

            if (Math.abs(robo.yerror) < robo.yfastmin) {
                if (Math.abs(robo.yerror) > robo.y_tolerance) {
                    if (robo.yerror > 0) {
                        robo.ypower = robo.yfastmin;
                    } else if (robo.yerror < 0) {
                        robo.ypower = -robo.yfastmin;
                    }
                } else {
                    robo.ypower = 0;
                }
            }

            if (Math.abs(robo.xerror) < robo.xfastmin) {
                if (Math.abs(robo.xerror) > robo.x_tolerance) {
                    if (robo.xerror > 0) {
                        robo.xpower = robo.xfastmin;
                    } else if (robo.xerror < 0) {
                        robo.xpower = -robo.xfastmin;
                    }
                } else {
                    robo.xpower = 0;
                }
            }

            if (Math.abs(robo.zerror) < robo.zfastmin) {
                if (Math.abs(robo.zerror) > robo.z_tolerance) {
                    if (robo.zerror > 0) {
                        robo.zpower = robo.zfastmin;
                    } else if (robo.zerror < 0) {
                        robo.zpower = -robo.zfastmin;
                    }
                } else {
                    robo.zpower = 0;
                }
            }
            //endregion

            //flip x and y powers when cross 90 threshold to correct
            if (Math.abs(robo.location.h) > 90) {
                robo.ypower = -robo.ypower;
                robo.xpower = -robo.xpower;
            }

            if ((robo.ztarget == robo.drive180.h) && (robo.headingdiff < 2)) {
                robo.zpower = 0;
            }

            robo.xp = robo.xpower;
            robo.yp = robo.ypower;

            if ((-120 < robo.location.h) && (robo.location.h < -60)) {
                robo.xpower = -robo.yp;
                robo.ypower = robo.xp;
            } else if ((120 > robo.location.h) && (robo.location.h > 60)) {
                robo.xpower = robo.yp;
                robo.ypower = -robo.xp;
            }

            Log.d("PositionCheck", "X power is: " + robo.xpower);
            Log.d("PositionCheck", "Y power is: " + robo.ypower);
            Log.d("PositionCheck", "Z power is: " + robo.zpower);

            robo.flpower = Range.clip(robo.ypower + robo.xpower - robo.zpower, -1, 1);
            robo.frpower = Range.clip(robo.ypower - robo.xpower + robo.zpower, -1, 1);
            robo.blpower = Range.clip(robo.ypower - robo.xpower - robo.zpower, -1, 1);
            robo.brpower = Range.clip(robo.ypower + robo.xpower + robo.zpower, -1, 1);

            //region - OTOS Logs
            Log.d("Test", "Y Value: " + robo.location.y);
            Log.d("Test", "X Value: " + robo.location.x);
            Log.d("Test", "Z Value: " + robo.location.h);
            //endregion
            //endregion

            //region - Apply Motor Power
            //if (drivetime.milliseconds() < timelimit) {
            robo.fl.setPower(robo.flpower);
            robo.fr.setPower(robo.frpower);
            robo.bl.setPower(robo.blpower);
            robo.br.setPower(robo.brpower);

            Log.d("PositionCheck", "FR power is: " + robo.frpower);
            Log.d("PositionCheck", "FL power is: " + robo.flpower);
            Log.d("PositionCheck", "BR power is: " + robo.brpower);
            Log.d("PositionCheck", "BL power is: " + robo.blpower);
            //endregion
        }
    }

    /*
    public void SampleAdjust(SparkFunOTOS.Pose2D driveto, double tolerance, boolean otos, boolean odo) {
        Log.d("PositionCheck", "Started Drive Method");

        robo.drivetarget = driveto;

        Log.d("PositionCheck", "Target = " + driveto.x + ", " + driveto.y + ", " + driveto.h);
        Log.d("PositionCheck", "XTarget: " + robo.drivetarget.x);

        //robo.odo.update();
        robo.xtarget = robo.drivetarget.x;
        //robo.odopos = robo.odo.getPosition();
        robo.otosloc = robo.otos.getPosition();

        /*if (otos && odo) {
            robo.location.x = (robo.otosloc.x + robo.odopos.getX(DistanceUnit.INCH)) / 2;
        } else if (otos) {
            robo.location.x = robo.otosloc.x;
        /*} else if (odo) {
            robo.location.x = robo.odopos.getX(DistanceUnit.INCH);
        }

        Log.d("PositionCheck", "X is: " + robo.location.x);
        Log.d("PositionCheck", "Otos X is: " + robo.otosloc.x);
        //Log.d("PositionCheck", "Odo X is: " + robo.odopos.getX(DistanceUnit.INCH));

        if ((Math.abs(robo.xtarget - robo.location.x)) > 0.25) {
            Log.d("PositionCheck", "In Drive Method Loop");

            //region - OTOS Drive
            //region - Universal Tracking
            if ((Math.abs(robo.xtarget - robo.location.x)) > tolerance) { //15
                if ((robo.xtarget - robo.location.x) > 0) {
                    robo.xerror = 1;
                } else if ((robo.xtarget - robo.location.x) < 0) {
                    robo.xerror = -1;
                }
            } else {
                robo.xerror = Range.clip(((robo.xtarget - robo.location.x) / tolerance), -1, 1); //15
            }

            Log.d("PositionCheck", "X error is: " + robo.xerror);

            robo.xpower = robo.xerror;

            if (Math.abs(robo.xerror) < robo.x_min) {
                if (Math.abs(robo.xerror) > robo.x_tolerance) {
                    if (robo.xerror > 0) {
                        robo.xpower = robo.x_min;
                    } else if (robo.xerror < 0) {
                        robo.xpower = -robo.x_min;
                    }
                } else {
                    robo.xpower = 0;
                }
            }

            //endregion

            //flip x and y powers when cross 90 threshold to correct
            if (Math.abs(robo.location.h) > 90) {
                robo.xpower = -robo.xpower;
            }

            robo.xp = robo.xpower;

            Log.d("PositionCheck", "X power is: " + robo.xpower);

            robo.flpower = Range.clip(robo.xpower, -1, 1);
            robo.frpower = Range.clip(-robo.xpower, -1, 1);
            robo.blpower = Range.clip(-robo.xpower, -1, 1);
            robo.brpower = Range.clip(robo.xpower, -1, 1);

            Log.d("Test", "X Value: " + robo.location.x);
            //endregion

            //region - Apply Motor Power
            robo.fl.setPower(robo.flpower);
            robo.fr.setPower(robo.frpower);
            robo.bl.setPower(robo.blpower);
            robo.br.setPower(robo.brpower);

            Log.d("PositionCheck", "FR power is: " + robo.frpower);
            Log.d("PositionCheck", "FL power is: " + robo.flpower);
            Log.d("PositionCheck", "BR power is: " + robo.brpower);
            Log.d("PositionCheck", "BL power is: " + robo.blpower);
            //endregion
        }
    }
*/

    public void SampleAdjust(SparkFunOTOS.Pose2D driveto, double tolerance, double rotation) {
        Log.d("PositionCheck", "Started Drive Method");

        robo.drivetarget = driveto;

        Log.d("PositionCheck", "Target = " + driveto.x + ", " + driveto.y + ", " + driveto.h);
        Log.d("PositionCheck", "XTarget: " + robo.drivetarget.x);

        if (robo.drivetarget.x != 0) {
            robo.xtarget = robo.drivetarget.x;
        } else {
            robo.xtarget = robo.otos.getPosition().x;
        }

        if (rotation != 0) {
            if (robo.xtarget > 0) {
                robo.xtarget = robo.drivetarget.x + 0.5;
            } else {
                robo.xtarget = robo.drivetarget.x - 0.25;
            }
        }

        robo.otosloc = robo.otos.getPosition();
        robo.location.x = robo.otosloc.x;

        Log.d("PositionCheck", "X is: " + robo.location.x);
        Log.d("PositionCheck", "Otos X is: " + robo.otosloc.x);

        if ((Math.abs(robo.xtarget - robo.location.x)) > 0.5) {
            Log.d("PositionCheck", "In Drive Method Loop");

            //region - OTOS Drive
            //region - Universal Tracking
            if ((Math.abs(robo.xtarget - robo.location.x)) > tolerance) { //15
                if ((robo.xtarget - robo.location.x) > 0) {
                    robo.xerror = 0.6;
                } else if ((robo.xtarget - robo.location.x) < 0) {
                    robo.xerror = -0.6;
                }
            } else {
                robo.xerror = Range.clip(((robo.xtarget - robo.location.x) / tolerance), -0.6, 0.6); //15
            }

            Log.d("PositionCheck", "X error is: " + robo.xerror);

            robo.xpower = robo.xerror;

            if (Math.abs(robo.xerror) < robo.x_min) {
                if (Math.abs(robo.xerror) > robo.x_tolerance) {
                    if (robo.xerror > 0) {
                        robo.xpower = robo.x_min;
                    } else if (robo.xerror < 0) {
                        robo.xpower = -robo.x_min;
                    }
                } else {
                    robo.xpower = 0;
                }
            }
            //endregion

            if ((robo.xpower > 0) && (robo.xpower < 0.35)) {
                robo.xpower = 0.35;
            } else if ((robo.xpower < 0) && (robo.xpower > (-0.35))) {
                robo.xpower = -0.35;
            }

            Log.d("PositionCheck", "X power is: " + robo.xpower);

            robo.flpower = Range.clip(robo.xpower, -0.6, 0.6);
            robo.frpower = Range.clip(-robo.xpower, -0.6, 0.6);
            robo.blpower = Range.clip(-robo.xpower, -0.6, 0.6);
            robo.brpower = Range.clip(robo.xpower, -0.6, 0.6);

            Log.d("Test", "X Value: " + robo.location.x);
            //endregion

            //region - Apply Motor Power
            robo.fl.setPower(robo.flpower);
            robo.fr.setPower(robo.frpower);
            robo.bl.setPower(robo.blpower);
            robo.br.setPower(robo.brpower);

            Log.d("PositionCheck", "FR power is: " + robo.frpower);
            Log.d("PositionCheck", "FL power is: " + robo.flpower);
            Log.d("PositionCheck", "BR power is: " + robo.brpower);
            Log.d("PositionCheck", "BL power is: " + robo.blpower);
            //endregion
        } else {
            robo.fl.setPower(0.2);
            robo.fr.setPower(0.2);
            robo.bl.setPower(0.2);
            robo.br.setPower(0.2);

            sleep(50);

            DriveStop();
        }
    }

    public void TURN(SparkFunOTOS.Pose2D turnto, SparkFunOTOS turnotos, /*GoBildaPinpointDriver turnodo,*/
                     boolean otos, boolean odo, double hpower, double hpwrlimit) {
        robo.drivetarget = turnto;

        robo.ztarget = robo.drivetarget.h;

        //turnodo.update();
        //robo.odopos = turnodo.getPosition();
        robo.otosloc = turnotos.getPosition();

        //region - Pos by AVG
        /*if (otos && odo) {
            robo.location.h = (robo.otosloc.h + robo.odopos.getHeading(AngleUnit.DEGREES)) / 2;
        } else if (odo) {
            robo.location.h = robo.odopos.getHeading(AngleUnit.DEGREES);
        } else if (otos) {*/
        robo.location.h = robo.otosloc.h;
        //}

        Log.d("PositionCheck", "Z is: " + robo.location.h);
        Log.d("PositionCheck", "Otos Z is: " + robo.otosloc.h);
        //Log.d("PositionCheck", "Odo Z is: " + robo.odopos.getHeading(AngleUnit.DEGREES));

        //region - Error Catch
        /*if ((robo.otosloc.h - robo.odopos.getHeading(AngleUnit.DEGREES)) > 1) {
            Log.d("Calculations", "Drive Error: H Opod = " + robo.odopos.getHeading(AngleUnit.DEGREES));
            Log.d("Calculations", "Drive Error: H OTOS = " + robo.otosloc.h);
        }*/
        //endregion
        //endregion

        if ((Math.abs(robo.ztarget - robo.location.h) > 1)) {
            //region - OTOS Drive
            //region - Universal Tracking
            if (robo.ztarget == robo.drive180.h) {
                if (robo.location.h > 0) {
                    robo.headingdiff = Math.abs(robo.ztarget - robo.location.h);
                } else if (robo.location.h < 0) {
                    robo.headingdiff = Math.abs(robo.ztarget + robo.location.h);
                }
            } else {
                robo.headingdiff = Math.abs(robo.ztarget - robo.location.h);
            }

            if (robo.headingdiff > 25) {
                if (!(Math.abs(robo.ztarget) == robo.drive180.h)) {
                    //if ((robo.ztarget - robo.location.h) > 0) {
                    robo.zerror = hpower;
                    /*} else if ((robo.ztarget - robo.location.h) < 0) {
                        robo.zerror = -hpower;
                    }*/
                } else {
                    if (robo.location.h > 0) {
                        robo.zerror = hpower;
                    } else if (robo.location.h < 0) {
                        robo.zerror = -hpower;
                    }
                }
            } else {
                if (!(robo.ztarget == robo.drive180.h)) {
                    robo.zerror = Range.clip(((robo.ztarget - robo.location.h) / 25), -hpwrlimit, hpwrlimit);
                } else {
                    if (robo.location.h > 0) {
                        robo.zerror = Range.clip(((robo.ztarget - robo.location.h) / 25), -hpwrlimit, hpwrlimit);
                    } else if (robo.location.h < 0) {
                        robo.zerror = Range.clip((-(robo.ztarget + robo.location.h) / 25), -hpwrlimit, hpwrlimit);
                    }
                }
            }

            Log.d("PositionCheck", "Z error is: " + robo.zerror);
            robo.zpower = robo.zerror;

            if (Math.abs(robo.zerror) < robo.z_min) {
                if (Math.abs(robo.zerror) > robo.z_tolerance) {
                    if (robo.zerror > 0) {
                        robo.zpower = robo.z_min;
                    } else if (robo.zerror < 0) {
                        robo.zpower = -robo.z_min;
                    }
                } else {
                    robo.zpower = 0;
                }
            }
            //endregion

            if ((robo.ztarget == robo.drive180.h) && (robo.headingdiff < 2)) {
                robo.zpower = 0;
            }

            Log.d("PositionCheck", "Z power is: " + robo.zpower);

            robo.flpower = Range.clip(-robo.zpower, -1, 1);
            robo.frpower = Range.clip(robo.zpower, -1, 1);
            robo.blpower = Range.clip(-robo.zpower, -1, 1);
            robo.brpower = Range.clip(robo.zpower, -1, 1);

            Log.d("Test", "Z Value: " + robo.location.h);
            //endregion

            //region - Apply Motor Power
            robo.fl.setPower(robo.flpower);
            robo.fr.setPower(robo.frpower);
            robo.bl.setPower(robo.blpower);
            robo.br.setPower(robo.brpower);

            Log.d("PositionCheck", "FR power is: " + robo.frpower);
            Log.d("PositionCheck", "FL power is: " + robo.flpower);
            Log.d("PositionCheck", "BR power is: " + robo.brpower);
            Log.d("PositionCheck", "BL power is: " + robo.blpower);
            //endregion
        }
    }

    public void Pos(SparkFunOTOS.Pose2D poscheck) {
        Log.d("PositionCheck", "Target (x, y, z): " + poscheck.x + ", " + poscheck.y + ", " + poscheck.h);
        Log.d("PositionCheck", "Current (x, y, z): " + robo.otos.getPosition().x + ", " +
                robo.otos.getPosition().y + ", " + robo.otos.getPosition().h);
    }

    public void DriveStop() {
        robo.fl.setPower(0);
        robo.fr.setPower(0);
        robo.bl.setPower(0);
        robo.br.setPower(0);
    }

    public void DriveHold(boolean forward) {
        if (forward) {
            robo.fl.setPower(0.2);
            robo.fr.setPower(0.2);
            robo.bl.setPower(0.2);
            robo.br.setPower(0.2);
        } else {
            robo.fl.setPower(-0.2);
            robo.fr.setPower(-0.2);
            robo.bl.setPower(-0.2);
            robo.br.setPower(-0.2);
        }

        sleep(50);

        DriveStop();
    }
    //endregion

    //region - Camera
    /*
    public void Scan() {
        robo.pixelpos.x = robo.otos.getPosition().x;
        robo.pixelpos.y = robo.otos.getPosition().y;
        robo.pixelpos.h = robo.otos.getPosition().h;

        // Read the current list
        List<ColorBlobLocatorProcessor.Blob> blobs = robo.colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(700, 6000, blobs);  // filter out very small blobs.

        // Filter blobs without modifying the original list to avoid ConcurrentModificationException
        List<ColorBlobLocatorProcessor.Blob> filteredBlobs = blobs.stream()
                .filter(blob -> blob.getBoxFit().center.y >= 30 && blob.getBoxFit().center.y <= 205)
                .collect(Collectors.toList());

        // Find the smallest blob by area from the filtered list
        ColorBlobLocatorProcessor.Blob targetBlob = filteredBlobs.stream()
                .min(Comparator.comparingDouble(ColorBlobLocatorProcessor.Blob::getContourArea))
                .orElse(null);

        robo.BLOB = targetBlob;

        if (robo.BLOB != null) {
            robo.box = robo.BLOB.getBoxFit();
        }

        if ((robo.box != null) && (robo.BLOB != null)) {
            if ((robo.box.angle < 25) || (robo.box.angle > 65)) {
                if (robo.box.size.width > robo.box.size.height) {
                    if (robo.box.angle < 25) {
                        robo.camrot = 90;
                    } else {
                        robo.camrot = 0;
                    }
                } else {
                    if (robo.box.angle < 25) {
                        robo.camrot = 0;
                    } else {
                        robo.camrot = 90;
                    }
                }
            } else if (robo.box.size.width > robo.box.size.height) {
                robo.camrot = 45;
            } else if (robo.box.size.height > robo.box.size.width) {
                robo.camrot = -45;
            } else {
                Log.d("Camera", "Angle Error");
            }

            if ((robo.box.center.y > 35) && (robo.box.center.y < 205)) {
                if (robo.camrot == 0) {
                    if (robo.box.size.width > robo.box.size.height) {
                        robo.contourbottom = robo.box.center.y + ((robo.box.size.width) / 2);

                        if (robo.box.center.x > 182) {
                            robo.contourinneredge = Range.clip((robo.box.center.x - (robo.box.size.height / 2)), 166, 320);
                        } else if (robo.box.center.x < 130) {
                            robo.contourinneredge = Range.clip((robo.box.center.x + (robo.box.size.height / 2)), 0, 154);
                        } else {
                            robo.contourinneredge = 160;
                        }
                    } else {
                        robo.contourbottom = robo.box.center.y + ((robo.box.size.height) / 2);

                        if (robo.box.center.x > 182) {
                            robo.contourinneredge = Range.clip((robo.box.center.x - (robo.box.size.width / 2)), 166, 320);
                        } else if (robo.box.center.x < 130) {
                            robo.contourinneredge = Range.clip((robo.box.center.x + (robo.box.size.width / 2)), 0, 154);
                        } else {
                            robo.contourinneredge = 160;
                        }
                    }
                } else if (robo.camrot == 90) {
                    if (robo.box.size.width > robo.box.size.height) {
                        robo.contourbottom = robo.box.center.y + ((robo.box.size.height) / 2);

                        if (robo.box.center.x > 183) {
                            robo.contourinneredge = Range.clip((robo.box.center.x - (robo.box.size.width / 2)), 166, 320);
                        } else if (robo.box.center.x < 128) {
                            robo.contourinneredge = Range.clip((robo.box.center.x + (robo.box.size.width / 2)), 0, 154);
                        } else {
                            robo.contourinneredge = 160;
                        }
                    } else {
                        robo.contourbottom = robo.box.center.y + ((robo.box.size.width) / 2);

                        if (robo.box.center.x > 183) {
                            robo.contourinneredge = Range.clip((robo.box.center.x - (robo.box.size.height / 2)), 166, 320);
                        } else if (robo.box.center.x < 128) {
                            robo.contourinneredge = Range.clip((robo.box.center.x + (robo.box.size.height / 2)), 0, 154);
                        } else {
                            robo.contourinneredge = 160;
                        }
                    }
                } else if ((robo.camrot == 45) || (robo.camrot == -45)) {
                    double x = Math.pow((robo.box.size.height / 2), 2);
                    double y = Math.pow((robo.box.size.width / 2), 2);
                    robo.contourbottom = robo.box.center.y + (Math.sqrt(x + y));

                    if (robo.box.center.x > 183) {
                        robo.contourinneredge = Range.clip((robo.box.center.x - (Math.sqrt(x + y))), 166, 320);
                    } else if (robo.box.center.x < 128) {
                        robo.contourinneredge = Range.clip((robo.box.center.x + (Math.sqrt(x + y))), 0, 154);
                    } else {
                        robo.contourinneredge = 160;
                    }
                }
            } else {
                Log.d("Camera", "Edges Error");
            }

            if (robo.camrot == 0) {
                robo.Lcamlim = robo.L0camlim + 18;
                robo.Rcamlim = robo.R0camlim - 18;
                robo.distdiff = 0;
            } else if (robo.camrot == 90) {
                robo.Lcamlim = robo.L0camlim + 35;
                robo.Rcamlim = robo.R0camlim - 35;
                robo.distdiff = -1;
            } else {
                robo.Lcamlim = robo.L0camlim + 30;
                robo.Rcamlim = robo.R0camlim - 30;
                robo.distdiff = 0;
            }

            if (robo.Lcamlim > 155) {
                robo.Lcamlim = 155;
            }

            if (robo.Rcamlim < 165) {
                robo.Rcamlim = 165;
            }

            robo.sampledist = ((0.000136) * (Math.pow(robo.contourbottom, 2))) - ((0.09707) *
                    (robo.contourbottom)) + 19.6094 + robo.distdiff;

            robo.disttoservo = ((0.00002169) * (Math.pow(robo.sampledist, 5))) - ((0.0008684) *
                    (Math.pow(robo.sampledist, 4))) + ((0.01386) * (Math.pow(robo.sampledist, 3))) -
                    ((0.1083) * (Math.pow(robo.sampledist, 2))) + ((0.4361) * (robo.sampledist)) - 0.5371;

            robo.pixtoinR = ((-0.00100539) * (Math.pow(robo.sampledist, 7))) + ((0.05703) *
                    (Math.pow(robo.sampledist, 6))) - ((1.3611) * (Math.pow(robo.sampledist, 5))) +
                    ((17.7115) * (Math.pow(robo.sampledist, 4))) - ((135.6876) * (Math.pow(robo.sampledist, 3)))
                    + ((612.1758) * (Math.pow(robo.sampledist, 2))) - ((1508.3958) * (robo.sampledist)) + 1595.3195;
            robo.pixtoinL = ((0.001571) * (Math.pow(robo.sampledist, 7))) - ((0.09287) *
                    (Math.pow(robo.sampledist, 6))) + ((2.3039) * (Math.pow(robo.sampledist, 5))) -
                    ((31.0487) * (Math.pow(robo.sampledist, 4))) + ((245.1624) * (Math.pow(robo.sampledist, 3)))
                    - ((1132.9033) * (Math.pow(robo.sampledist, 2))) + ((2833.5406) * (robo.sampledist)) - 2931.5921;

            if ((robo.contourinneredge > robo.Lcamlim) && (robo.contourinneredge < robo.Rcamlim)) {
                robo.offdist = 0;
                robo.needtostrafe = false;
                robo.pixelpos.x = 0;
            } else if (robo.contourinneredge < robo.Lcamlim) {
                robo.offdist = robo.Lcamlim - robo.contourinneredge;
                robo.offpixels = Math.abs(160 - robo.contourinneredge) + 25;
                robo.pixeltoindistL = robo.offpixels / robo.pixtoinL;
                robo.needtostrafe = true;
                robo.pixelpos.x = Range.clip((-robo.pixeltoindistL), (robo.otos.getPosition().x - 2), (robo.otos.getPosition().x + 2));
            } else {
                robo.offdist = robo.contourinneredge - robo.Rcamlim;
                robo.offpixels = Math.abs(160 - robo.contourinneredge) + 25;
                robo.pixeltoindistR = robo.offpixels / robo.pixtoinR;
                robo.needtostrafe = true;
                robo.pixelpos.x = Range.clip(robo.pixeltoindistR, (robo.otos.getPosition().x - 2), (robo.otos.getPosition().x + 2));
            }

            Log.d("Camera", "Pos to go to: " + robo.pixelpos.x);
        } else {
            robo.pixelpos.x = robo.otosloc.x;
            Log.d("Camera", "Null Blocks");
        }
    }
    */

    public void YellowGrab(double trylimit) {
        DriveStop();

        drivetimeout = runtime.milliseconds() + 500;
        while (((Math.abs(robo.otos.getVelocity().x + robo.otos.getVelocity().y + robo.otos.getVelocity().h)) > 0.5) && (runtime.milliseconds() < drivetimeout)) {
            idle();
        }

        /*scantime = runtime.milliseconds() + 400;
        while (opModeIsActive() && (scantime > runtime.milliseconds())) {
            Scan();
        }*/

        /*drivetimeout = runtime.milliseconds() + 400;
        while (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && (Math.abs(robo.pixelpos.x - robo.location.x) > 1)) {
            SampleAdjust(robo.pixelpos, 10, robo.camrot); //XYZ?

            Log.d("Post", "Current1: " + robo.otos.getPosition().x + ", " + robo.otos.getPosition().y
                    + ", " + robo.otos.getPosition().h);
            Log.d("Post", "Pixel1: " + robo.pixelpos.x + ", " + robo.pixelpos.y + ", " + robo.pixelpos.h);
        }

        DriveStop();
        Pos(robo.pixelpos);*/

        robo.inExtL.setPosition(Range.clip((robo.intakeleftin - robo.disttoservo - 0.01), robo.intakeleftout, robo.intakeleftin));
        robo.inExtR.setPosition(Range.clip((robo.intakerightin + robo.disttoservo + 0.01), robo.intakerightin, robo.intakerightout));
        IntakePivot(robo.intakepivotdown);

        sleep(200);

        if (robo.camrot == 0) {
            IntakeWrist(robo.intakewrist0);
        } else if (robo.camrot == -45) {
            IntakeWrist(robo.intakewrist_45);
        } else if (robo.camrot == 45) {
            IntakeWrist(robo.intakewrist45);
        } else if (robo.camrot == 90) {
            IntakeWrist(robo.intakewrist90);
        }

        if (robo.pin0.getState() && robo.pin1.getState()) {
            robo.colorseen = "Yellow";
        } else if (robo.pin0.getState()) {
            robo.colorseen = "Blue";
        } else if (robo.pin1.getState()) {
            robo.colorseen = "Red";
        } else {
            robo.colorseen = "None";
        }

        Log.d("Color", "Color Seen: " + robo.colorseen);

        yellowcount = 0;
        drivetimeout = runtime.milliseconds() + 700;
        while ((!(robo.colorseen.matches("Yellow"))) && (runtime.milliseconds() < drivetimeout) && (yellowcount < 4)) {
            if (robo.pin0.getState() && robo.pin1.getState()) {
                robo.colorseen = "Yellow";
                yellowcount = yellowcount + 1;
            } else if (robo.pin0.getState()) {
                robo.colorseen = "Blue";
                yellowcount = 0;
            } else if (robo.pin1.getState()) {
                robo.colorseen = "Red";
                yellowcount = 0;
            } else {
                robo.colorseen = "None";
                yellowcount = 0;
            }

            Log.d("Color", "Color1 Seen: " + robo.colorseen);

            robo.fl.setPower(-0.45);
            robo.fr.setPower(0.5);
            robo.bl.setPower(0.5);
            robo.br.setPower(-0.45);
        }

        DriveStop();

        Intake(robo.intakeclosed);

        sleep(200);

        if (robo.pin0.getState() && robo.pin1.getState()) {
            robo.colorseen = "Yellow";
        } else if (robo.pin0.getState()) {
            robo.colorseen = "Blue";
        } else if (robo.pin1.getState()) {
            robo.colorseen = "Red";
        } else {
            robo.colorseen = "None";
        }

        Log.d("Color", "Color2 Seen: " + robo.colorseen);

        if (robo.yellowresetcount >= (trylimit + 1)) {
            hassample = false;

            YellowDropPark();
            park = true;
        } else if (!(robo.colorseen.matches("Yellow"))) {
            hassample = false;

            YellowGrabReset(robo.yellowresetcount);
            robo.yellowresetcount = robo.yellowresetcount + 1;

            YellowGrab(trylimit);
        } else {
            IntakePivot(robo.intakepivotback);
            IntakeExt(robo.intakeleftin, robo.intakerightin);
            IntakeWrist(robo.intakewrist0);

            if (!(robo.colorseen.matches("Yellow"))) {
                hassample = false;

                YellowGrabReset(robo.yellowresetcount);
                robo.yellowresetcount = robo.yellowresetcount + 1;

                YellowGrab(trylimit);
            } else {
                ClawRot(robo.clawrotatetransfer);

                drivetimeout = runtime.milliseconds() + 1100;
                robo.loopcheck = true;
                transfer = false;
                lift = false;
                while (opModeIsActive() && (((runtime.milliseconds() < drivetimeout) && (((Math.abs(robo.postsub.x -
                        robo.location.x) > 0.25)) || (Math.abs(robo.postsub.y - robo.location.y) > 0.25) ||
                        (Math.abs(robo.postsub.h - robo.location.h) > 0.25))) || (!lift))) {
                    XYZ(robo.postsub, 2, true, false, 0.5); //XYZ?

                    IntakeExt(robo.intakeleftin, robo.intakerightin);
                    IntakePivot(robo.intakepivotback);
                    IntakeWrist(robo.intakewrist0);

                    if (!robo.clawbeam.getState() && robo.loopcheck) {
                        grabtime = runtime.milliseconds();
                        robo.loopcheck = false;
                        transfer = true;
                    }

                    if (transfer && (robo.colorseen.matches("Yellow"))) {
                        if (runtime.milliseconds() >= (grabtime + 50)) {
                            Claw(robo.clawclosed);

                            if (runtime.milliseconds() >= (grabtime + 250)) {
                                Intake(robo.intakeopen);

                                transfer = false;
                                lift = true;
                                hassample = true;
                            }
                        }
                    } else if (!robo.colorseen.matches("Yellow")) {
                        hassample = false;

                        YellowGrabReset(robo.yellowresetcount);
                        robo.yellowresetcount = robo.yellowresetcount + 1;

                        YellowGrab(trylimit);
                    }

                    if (robo.pin0.getState() && robo.pin1.getState()) {
                        robo.colorseen = "Yellow";
                    } else if (robo.pin0.getState()) {
                        robo.colorseen = "Blue";
                    } else if (robo.pin1.getState()) {
                        robo.colorseen = "Red";
                    } else {
                        robo.colorseen = "None";
                    }
                }

                Pos(robo.postsub);

                armmove = true;
                drivetimeout = runtime.milliseconds() + 1300; //800
                while ((robo.lift1.getCurrentPosition() < robo.lifthigh) || (runtime.milliseconds() < drivetimeout)) {
                    if (opModeIsActive() && (((Math.abs(robo.deliver1.x - robo.location.x) > 0.25)) ||
                            (Math.abs(robo.deliver1.y - robo.location.y) > 0.25) ||
                            (Math.abs(robo.deliver1.h - robo.location.h) > 0.25))) {
                        XYZ(robo.deliver1, 10, true, false, 0.5); //XYZ?
                    }

                    LiftMulti(robo.lifthigh, runtime.milliseconds());

                    if (armmove) {
                        ServoArm(robo.armlup, robo.armrup);
                        ClawRot(robo.clawrotatedeliverb);
                        armmove = false;
                    }
                }

                DriveStop();
                LiftStop(robo.lifthigh);
                Pos(robo.deliver1);
                lift = false;
            }
        }
    }

    public void YellowGrabReset(double resetcount) {
        IntakeExt(robo.intakeleftin, robo.intakerightin);
        IntakePivot(robo.intakepivotscan);
        Intake(robo.intakeopen);
        IntakeWrist(robo.intakewrist0);
        ClawRot(robo.clawrotatescan);

        if (resetcount == 0) {
            robo.subgrab = robo.grab3;
        } else if (resetcount == 1) {
            robo.subgrab = robo.grab4;
        } else if (resetcount == 2) {
            robo.subgrab = robo.grab2;
        } else {
            robo.subgrab = robo.park;
        }

        drivetimeout = runtime.milliseconds() + 500;
        while (opModeIsActive() && (((Math.abs(robo.subgrab.x - robo.location.x) > 0.25)) || (Math.abs(robo.subgrab.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.subgrab.h - robo.location.h) > 0.25)) && (runtime.milliseconds() < drivetimeout)) {
            XYZ(robo.subgrab, 1, true, false, 0.5);
        }

        DriveStop();
        Pos(robo.subgrab);

        IntakePivot(robo.intakepivotscan);
        ClawRot(robo.clawrotatescan);
    }

    public void ColorGrab(String colorban) {
        while ((Math.abs(robo.otos.getVelocity().x + robo.otos.getVelocity().y + robo.otos.getVelocity().h)) > 0.5) {
            idle();
        }

        //Scan();

        robo.inExtL.setPosition(Range.clip((robo.intakeleftin - robo.disttoservo), robo.intakeleftout, robo.intakeleftin));
        robo.inExtR.setPosition(Range.clip((robo.intakerightin + robo.disttoservo), robo.intakerightin, robo.intakerightout));
        IntakePivot(robo.intakepivotdown);

        if (robo.camrot == 0) {
            IntakeWrist(robo.intakewrist0);
        } else if (robo.camrot == -45) {
            IntakeWrist(robo.intakewrist_45);
        } else if (robo.camrot == 45) {
            IntakeWrist(robo.intakewrist45);
        } else if (robo.camrot == 90) {
            IntakeWrist(robo.intakewrist90);
        }

        sleep(200);

        Intake(robo.intakeclosed);

        sleep(200);

        if (robo.pin0.getState() && robo.pin1.getState()) {
            robo.colorseen = "Yellow";
        } else if (robo.pin0.getState()) {
            robo.colorseen = "Blue";
        } else if (robo.pin1.getState()) {
            robo.colorseen = "Red";
        } else {
            robo.colorseen = "None";
        }

        if ((robo.colorseen.matches("None")) || (robo.colorseen.matches(colorban)) ||
                (robo.colorseen.matches("Yellow"))) {
            Intake(robo.intakeopen);
            hascolor = false;
        } else {
            Intake(robo.intakeclosed);

            sleep(200);

            hascolor = true;
        }

        IntakeWrist(robo.intakewrist0);
        IntakePivot(robo.intakepivotback);
        IntakeExt(robo.intakeleftin, robo.intakerightin);
    }
    //endregion

    //region - Lift
    public void Lift(double liftupto, double uptime) {
        while ((robo.lift1.getCurrentPosition() < liftupto) && (uptime < (uptime + 1000))) {
            robo.lift1.setPower(1);
            robo.lift2.setPower(1);
            robo.lift3.setPower(1);

            Log.d("LIFT", "Lift Pos: " + robo.lift1.getCurrentPosition());
        }

        sleep(100);

        LiftStop(liftupto);
    }

    public void LiftMulti(double liftupto, double uptime) {
        if ((robo.lift1.getCurrentPosition() < liftupto) && (uptime < (uptime + 2000))) {
            robo.lift1.setPower(1);
            robo.lift2.setPower(1);
            robo.lift3.setPower(1);

            Log.d("LIFT", "Lift Pos: " + robo.lift1.getCurrentPosition());
        } else {
            LiftStop(liftupto);
        }
    }

    public void Liftdown(double downtime) {
        while ((robo.lift1.getCurrentPosition() > robo.liftdown) && (robo.liftMag.getState()) && (downtime < (downtime + 1000))) {
            robo.lift1.setPower(-1);
            robo.lift2.setPower(-1);
            robo.lift3.setPower(-1);
        }

        LiftStop(robo.liftdown);
    }

    public void LiftdownMulti(double downtime) {
        if ((robo.lift1.getCurrentPosition() > robo.liftdown) && (robo.liftMag.getState()) && (downtime < (downtime + 1000))) {
            robo.lift1.setPower(-1);
            robo.lift2.setPower(-1);
            robo.lift3.setPower(-1);
        } else {
            LiftStop(robo.liftdown);
        }
    }

    public void LiftStop(double liftpos) {
        if (liftpos > 50) {
            robo.lift1.setPower(robo.liftholdpower);
            robo.lift2.setPower(robo.liftholdpower);
            robo.lift3.setPower(robo.liftholdpower);
        } else {
            robo.lift1.setPower(0);
            robo.lift2.setPower(0);
            robo.lift3.setPower(0);
        }
    }

    public void ServoArm(double armposl, double armposr) {
        robo.arml.setPosition(armposl);
        robo.armr.setPosition(armposr);
    }

    public void Claw(double clawpos) {
        robo.claw.setPosition(clawpos);
    }

    public void ClawRot(double clawrotpos) {
        robo.clawrotate.setPosition(clawrotpos);
    }
    //endregion

    //region - Intake
    public void IntakeExt(double Lintakegoto, double Rintakegoto) {
        robo.inExtL.setPosition(Lintakegoto);
        robo.inExtR.setPosition(Rintakegoto);
    }

    public void Intake(double intakeposition) {
        robo.intakeclaw.setPosition(intakeposition);
    }

    public void IntakePivot(double intakepivotpos) {
        robo.intakepivot.setPosition(intakepivotpos);
    }

    public void IntakeWrist(double intakewristpos) {
        robo.intakewrist.setPosition(intakewristpos);
    }
    //endregion

    //region - Left
    public void DeliverLeft(boolean lift) {
        //region - CycleLeft
        DriveStop();

        if (lift) {
            Lift(robo.lifthigh, runtime.milliseconds());

            sleep(300);
        }

        ServoArm(robo.armldeliver, robo.armrdeliver);
        Claw(robo.clawopenfull);

        sleep(100);

        ClawRot(robo.clawrotatescan);
        //ServoArm(robo.armservoprefront, robo.armservoprefront);
        //endregion
    }

    public void LeftBarcodeGrab(SparkFunOTOS.Pose2D drivetoposleft, double intakewristpos, boolean thirdgrab) {
        //region - Drivetoposleft
        IntakePivot(robo.intakepivotscan);
        Intake(robo.intakeopen);
        ClawRot(robo.clawrotatescan);
        Claw(robo.clawopenfull);

        drivetimeout = runtime.milliseconds() + 1000;
        scantime = runtime.milliseconds() + 1000;
        intake = true;
        robo.loopcheck = false;
        stopdrive = true;
        extradrivetime = 700;
        grabbing = true;
        closemark = true;
        afterfix = true;
        while (((robo.lift1.getCurrentPosition() > robo.liftdown) || (!robo.loopcheck)) && ((runtime.milliseconds() <
                (drivetimeout + 800 + extradrivetime)) || (runtime.milliseconds() > (scantime + 1700)) || grabbing) && opModeIsActive()) {
            if ((runtime.milliseconds() < (drivetimeout + 100)) && (((Math.abs(drivetoposleft.x - robo.location.x) > 0.25))
                    || (Math.abs(drivetoposleft.y - robo.location.y) > 0.25) || (Math.abs(drivetoposleft.h - robo.location.h) > 0.25))) {
                Log.d("Test", "In Grab Drive 1 pt1");

                XYZ(drivetoposleft, 15, true, false, 0.6); //FAST?

                Log.d("Test", "In Grab Drive 1 pt2");
            } else if (stopdrive) {
                DriveHold(false);
                DriveStop();
                stopdrive = false;
                sampleadjust = false;
            }

            if (intake && (!stopdrive) && (runtime.milliseconds() >= drivetimeout)) {
                if (((Math.abs(robo.otos.getVelocity().x + robo.otos.getVelocity().y + robo.otos.getVelocity().h)) >
                        0.5) && (robo.lift1.getCurrentPosition() > 5)) {
                    idle();
                    scantime = runtime.milliseconds();
                    lastpos = robo.otos.getPosition();
                } else {
                    /*if ((scantime + 400) > runtime.milliseconds()) {
                        Scan();
                    } else*/ if (!sampleadjust) {
                        adjust = true;
                        sampleadjust = true;
                    }

                    Log.d("Adjust", "Inches to go: " + robo.pixelpos.x);
                    Log.d("Adjust", "Pos: " + robo.otos.getPosition().x);

                    /*if ((runtime.milliseconds() < (scantime + 900)) && /*(Math.abs(robo.pixelpos.x) > 0.5) &&/
                            ((Math.abs((lastpos.x + robo.pixelpos.x) - robo.otos.getPosition().x)) > 0.5) &&
                            ((Math.abs((lastpos.x + robo.pixelpos.x) - robo.otos.getPosition().x)) < 3) && adjust) {
                        SampleAdjust(robo.pixelpos, 10, robo.camrot); //XYZ?

                        Log.d("Post", "Current: " + robo.otos.getPosition().x + ", " + robo.otos.getPosition().y
                                + ", " + robo.otos.getPosition().h);
                        Log.d("Post", "Pixel: " + robo.pixelpos.x + ", " + robo.pixelpos.y + ", " + robo.pixelpos.h);
                    } else if (runtime.milliseconds() > (scantime + 550)) {
                        sampleadjust = true;
                        adjust = false;
                    }*/

                    if (sampleadjust) {
                        if (adjust) {
                            DriveStop();
                            Pos(robo.pixelpos);
                            adjust = false;
                        }

                        robo.inExtL.setPosition(Range.clip((robo.intakeleftin - robo.disttoservo), robo.intakeleftout, robo.intakeleftin));
                        robo.inExtR.setPosition(Range.clip((robo.intakerightin + robo.disttoservo), robo.intakerightin, robo.intakerightout));
                        IntakePivot(robo.intakepivotdown);
                        ClawRot(robo.clawrotatetransfer);
                        Claw(robo.clawautograb);

                        if (runtime.milliseconds() > (scantime + 600)) {
                            IntakeWrist(intakewristpos);

                            if (robo.pin0.getState() && robo.pin1.getState()) {
                                robo.colorseen = "Yellow";
                            } else if (robo.pin0.getState()) {
                                robo.colorseen = "Blue";
                            } else if (robo.pin1.getState()) {
                                robo.colorseen = "Red";
                            } else {
                                robo.colorseen = "None";
                            }

                            if ((runtime.milliseconds() > (scantime + 800)) && (robo.colorseen.matches("Yellow"))) {
                                if (afterfix) {
                                    robo.fl.setPower(0.3);
                                    robo.fr.setPower(-0.3);
                                    robo.bl.setPower(-0.3);
                                    robo.br.setPower(0.3);

                                    sleep(50);

                                    afterfix = false;
                                }

                                DriveStop();

                                Intake(robo.intakeclosed);

                                if (closemark) {
                                    closetime = runtime.milliseconds() + 700;
                                    closemark = false;
                                }

                                if (runtime.milliseconds() > (closetime - 500)) {
                                    IntakePivot(robo.intakepivotback);
                                    IntakeExt(robo.intakeleftin, robo.intakerightin);
                                    IntakeWrist(robo.intakewrist0);
                                    robo.loopcheck = true;
                                    intake = false;
                                    sampleadjust = false;
                                    grabbing = false;
                                }
                            } else {
                                if (!thirdgrab) {
                                    robo.fl.setPower(-0.4);
                                    robo.fr.setPower(0.4);
                                    robo.bl.setPower(0.4);
                                    robo.br.setPower(-0.4);
                                } else {
                                    robo.fl.setPower(-0.4);
                                    robo.fr.setPower(0.55);
                                    robo.bl.setPower(0.55);
                                    robo.br.setPower(-0.4);
                                }
                            }
                        }
                    }
                }
            }

            if (runtime.milliseconds() > (drivetimeout - 900)) {
                LiftdownMulti(runtime.milliseconds());
            }

            if ((drivetimeout - runtime.milliseconds()) < 800) {
                ServoArm(robo.armltransfer, robo.armrtransfer);
            }
        }

        DriveStop();
        LiftStop(robo.liftdown);
        Pos(drivetoposleft);

        Claw(robo.clawautograb);

        transfer = false;
        lift = false;
        clawgrab = false;
        drivetimeout = runtime.milliseconds() + 1700; //1300
        while ((robo.lift1.getCurrentPosition() < robo.lifthigh) || (runtime.milliseconds() < drivetimeout)) {
            if (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && (((Math.abs(robo.deliver1.x -
                    robo.location.x) > 0.25)) || (Math.abs(robo.deliver1.y - robo.location.y) > 0.25) ||
                    (Math.abs(robo.deliver1.h - robo.location.h) > 0.25))) {
                XYZ(robo.deliver1, 8, true, false, 0.6); //FAST?
            } else {
                DriveStop();
            }

            if (runtime.milliseconds() > closetime) {
                Claw(robo.clawclosed);
            }

            if (lift) {
                LiftMulti(robo.lifthigh, runtime.milliseconds());
            }

            if (!robo.clawbeam.getState() && robo.loopcheck) {
                grabtime = runtime.milliseconds();
                robo.loopcheck = false;
                transfer = true;
            }

            if (transfer) {
                if (runtime.milliseconds() >= (grabtime + 250)) {
                    Claw(robo.clawclosed);

                    if (runtime.milliseconds() >= (grabtime + 450)) {
                        Intake(robo.intakeopen);
                        Claw(robo.clawclosed);

                        if (runtime.milliseconds() >= (grabtime + 550)) {
                            ServoArm(robo.armlup, robo.armrup);
                            ClawRot(robo.clawrotatedeliverb);
                            Claw(robo.clawclosed);
                            lift = true;
                            transfer = false;
                        }
                    }
                }
            }
        }

        DriveStop();
        LiftStop(robo.lifthigh);
        Pos(robo.deliver1);
        lift = false;
        //endregion
    }

    public void ParkLeft() {
        //region - LeftPark
        IntakePivot(robo.intakepivotback);
        IntakeExt(robo.intakeleftin, robo.intakerightin);

        drivetimeout = runtime.milliseconds() + 1000;
        while ((robo.lift1.getCurrentPosition() > robo.liftdown) || (runtime.milliseconds() < drivetimeout)) {
            if (opModeIsActive() && (((Math.abs(robo.presub.x - robo.location.x) > 0.25)) || (Math.abs(robo.presub.y - robo.location.y) > 0.25) ||
                    (Math.abs(robo.presub.h - robo.location.h) > 0.25))) {
                FAST(robo.presub, true, false, 0.5); //XYZ?
            }

            LiftdownMulti(runtime.milliseconds());
        }

        DriveStop();
        LiftStop(robo.liftdown);
        Pos(robo.presub);

        ServoArm(robo.armlup, robo.armrup);
        ClawRot(robo.clawflat);

        drivetimeout = runtime.milliseconds() + 500;
        while ((runtime.milliseconds() < drivetimeout) && opModeIsActive() && (((Math.abs(robo.park.x -
                robo.location.x) > 0.25)) || (Math.abs(robo.park.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.park.h - robo.location.h) > 0.25))) {
            FAST(robo.park, true, false, 0.5); //XYZ?
        }

        DriveStop();
        Pos(robo.park);

        ServoArm(robo.armlpark, robo.armrpark);

        sleep(200);
        //endregion
    }

    public void YellowDropPark() {
        ServoArm(robo.armlup, robo.armrup);
        ClawRot(robo.clawrotatedown);
        Claw(robo.clawopenfull);
        Intake(robo.intakeclosed);

        sleep(100);

        ClawRot(robo.clawflat);

        sleep(100);

        ServoArm(robo.armlpark, robo.armrpark);
        IntakeExt(robo.intakeleftin, robo.intakerightin);
        IntakePivot(robo.intakepivotback);
        IntakeWrist(robo.intakewrist0);
    }

    public void LSubGrab(double attempts) {
        IntakeExt(robo.intakeleftin, robo.intakerightin);
        IntakePivot(robo.intakepivotback);
        Intake(robo.intakeopen);
        IntakeWrist(robo.intakewrist0);

        drivetimeout = runtime.milliseconds() + 1300;
        while ((robo.lift1.getCurrentPosition() > robo.liftdown) || (runtime.milliseconds() < drivetimeout)) {
            if (opModeIsActive() && (((Math.abs(robo.presub.x - robo.location.x) > 0.25)) || (Math.abs(robo.presub.y - robo.location.y) > 0.25) ||
                    (Math.abs(robo.presub.h - robo.location.h) > 0.25))) {
                XYZ(robo.presub, 2, true, false, 0.4); //XYZ?
            }

            LiftdownMulti(runtime.milliseconds());

            if ((drivetimeout - runtime.milliseconds()) < 1500) {
                ServoArm(robo.armltransfer, robo.armrtransfer);
            }
        }

        LiftStop(robo.liftdown);
        Pos(robo.presub);

        drivetimeout = runtime.milliseconds() + 800;
        while ((runtime.milliseconds() < drivetimeout) && opModeIsActive() && (((Math.abs(robo.park.x -
                robo.location.x) > 0.25)) || (Math.abs(robo.park.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.park.h - robo.location.h) > 0.25))) {
            XYZ(robo.park, 2, true, false, 0.6); //XYZ?
        }

        DriveStop();
        Pos(robo.park);

        IntakePivot(robo.intakepivotscan);
        ClawRot(robo.clawrotatescan);

        YellowGrab(attempts);
    }
    //endregion

    //region - Right
    public void DeliverRightBar(double drive1, String colordrop) {
        //region - RightBarDeliver
        //region - Determine Drive
        if (drive1 == 1) {
            rightdrive1 = robo.Rbar;
            rtolerance = 20;
        } else if (drive1 == 2) {
            rightdrive1 = robo.Rbar2;
            rtolerance = 20;
        } else if (drive1 == 3) {
            rightdrive1 = robo.Rbar3;
            rtolerance = 20;
        } else if (drive1 == 4) {
            rightdrive1 = robo.Rbar4;
            rtolerance = 20;
        }
        //endregion

        IntakePivot(robo.intakepivotback);
        IntakeExt(robo.intakeleftin, robo.intakerightin);

        ServoArm(robo.armltransfer, robo.armrtransfer);

        ClawRot(robo.clawrotatebar2);

        drivetimeout = runtime.milliseconds() + 1200;
        while ((robo.lift1.getCurrentPosition() < liftupto) || (runtime.milliseconds() < drivetimeout)) {
            if (opModeIsActive() && (((Math.abs(robo.leftdrivebucket.x - robo.location.x) > 0.25)) || (Math.abs(robo.leftdrivebucket.y - robo.location.y) > 0.25) ||
                    (Math.abs(robo.leftdrivebucket.h - robo.location.h) > 0.25))) {
                XYZ(rightdrive1, rtolerance, true, false, 0.15);
            }

            LiftMulti(robo.Lbar2, runtime.milliseconds());
        }

        DriveStop();
        LiftStop(robo.lifthigh);
        Pos(rightdrive1);

        if (drive1 == 1) {
            IntakePivot(robo.intakepivotscan);
            ClawRot(robo.clawrotatescan);
        }

        Claw(robo.clawopenfull);

        sleep(200);

        if (drive1 == 1) {
            ColorGrab(colordrop);

            if (hascolor) {
                drivetimeout = runtime.milliseconds() + 1200;
                while (runtime.milliseconds() < drivetimeout) {
                    if (opModeIsActive() && (((Math.abs(robo.Rtrade1.x - robo.location.x) > 0.25)) ||
                            (Math.abs(robo.Rtrade1.y - robo.location.y) > 0.25) ||
                            (Math.abs(robo.Rtrade1.h - robo.location.h) > 0.25))) {
                        XYZ(robo.Rtrade1, rtolerance, true, false, 0.15);
                    }

                    if (runtime.milliseconds() > (drivetimeout - 900)) {
                        ServoArm(robo.armltransfer, robo.armrtransfer);
                        ClawRot(robo.clawrotatetransfer);
                        Claw(robo.clawautograb);

                        if (!robo.clawbeam.getState() && (runtime.milliseconds() > (drivetimeout - 800))) {
                            Claw((robo.clawclosed));

                            if (runtime.milliseconds() > (drivetimeout - 600)) {
                                Intake(robo.intakeopen);

                                if (runtime.milliseconds() > (drivetimeout - 400)) {
                                    ClawRot(robo.clawrotatebackgrab);
                                    ServoArm(robo.armlup, robo.armrup);
                                }
                            }
                        }
                    }
                }

                //ServoArm(robo.armldeliver, robo.armrdeliver);

                DriveStop();
                Pos(robo.Rtrade1);

                /*sleep(400);

                ServoArm(robo.armservoback1, robo.armservoback1);

                sleep(150);

                ServoArm(robo.armservoback2, robo.armservoback2);

                sleep(150);*/

                ServoArm(robo.armlbackgrab, robo.armrbackgrab);
                Claw(robo.clawautograb);

                drivetimeout = runtime.milliseconds() + 1000;
                while (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && robo.clawbeam.getState()) {
                    if (((Math.abs(robo.Rtrade2.x - robo.location.x) > 0.25)) ||
                            (Math.abs(robo.Rtrade2.y - robo.location.y) > 0.25) ||
                            (Math.abs(robo.Rtrade2.h - robo.location.h) > 0.25)) {
                        XYZ(robo.Rtrade2, rtolerance, true, false, 0.15);
                    }
                }

                DriveStop();
                Pos(robo.Rtrade2);

                Claw(robo.clawclosed);

                sleep(200);

                ClawRot(robo.clawrotatetransfer);

                DeliverRightBar(2, colordrop);
            }
        }

        ClawRot(robo.clawrotatetransfer);
        ServoArm(robo.armltransfer, robo.armrtransfer);
        //endregion
    }

    public void RMove() {
        drivetimeout = runtime.milliseconds() + 1200;
        while ((robo.lift1.getCurrentPosition() > robo.liftdown) || (runtime.milliseconds() < drivetimeout)) {
            if (opModeIsActive() && (((Math.abs(robo.Rgrab1.x - robo.location.x) > 0.25)) ||
                    (Math.abs(robo.Rgrab1.y - robo.location.y) > 0.25) ||
                    (Math.abs(robo.Rgrab1.h - robo.location.h) > 0.25))) {
                XYZ(robo.Rgrab1, rtolerance, true, false, 0.15);
            }

            if (runtime.milliseconds() > (drivetimeout - 400)) {
                LiftdownMulti(runtime.milliseconds());
            }

            if (runtime.milliseconds() > (drivetimeout - 400)) {
                IntakeExt(robo.intakeleftout, robo.intakerightout);
                IntakePivot(robo.intakepivotdown);
                IntakeWrist(robo.intakewrist45);
                Intake(robo.intakeopen);
            }
        }

        DriveStop();
        LiftStop(robo.liftdown);
        Pos(robo.Rgrab1);

        Intake(robo.intakeclosed);

        sleep(200);

        drivetimeout = runtime.milliseconds() + 800;
        while (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && (((Math.abs(robo.Rmove1.x -
                robo.location.x) > 0.25)) || (Math.abs(robo.Rmove1.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.Rmove1.h - robo.location.h) > 0.25))) {
            XYZ(robo.Rmove1, rtolerance, true, false, 0.15);

            IntakeWrist(robo.intakewrist_45);
        }

        DriveStop();
        Pos(robo.Rmove1);

        Intake(robo.intakeopen);

        sleep(200);

        drivetimeout = runtime.milliseconds() + 800;
        while (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && (((Math.abs(robo.Rgrab2.x -
                robo.location.x) > 0.25)) || (Math.abs(robo.Rgrab2.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.Rgrab2.h - robo.location.h) > 0.25))) {
            XYZ(robo.Rgrab2, rtolerance, true, false, 0.15);

            IntakeWrist(robo.intakewrist45);
        }

        DriveStop();
        Pos(robo.Rgrab2);

        Intake(robo.intakeclosed);

        sleep(200);

        drivetimeout = runtime.milliseconds() + 800;
        while (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && (((Math.abs(robo.Rmove2.x -
                robo.location.x) > 0.25)) || (Math.abs(robo.Rmove2.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.Rmove2.h - robo.location.h) > 0.25))) {
            XYZ(robo.Rmove2, rtolerance, true, false, 0.15);

            IntakeWrist(robo.intakewrist_45);
        }

        DriveStop();
        Pos(robo.Rmove2);

        Intake(robo.intakeopen);

        sleep(200);

        drivetimeout = runtime.milliseconds() + 800;
        while (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && (((Math.abs(robo.Rgrab3.x -
                robo.location.x) > 0.25)) || (Math.abs(robo.Rgrab3.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.Rgrab3.h - robo.location.h) > 0.25))) {
            XYZ(robo.Rgrab3, rtolerance, true, false, 0.15);

            IntakeWrist(robo.intakewrist45);
        }

        DriveStop();
        Pos(robo.Rgrab3);

        Intake(robo.intakeclosed);

        sleep(200);

        loopcount = true;
        drivetimeout = runtime.milliseconds() + 1000;
        while ((runtime.milliseconds() < drivetimeout) && opModeIsActive()) {
            if (opModeIsActive() && (((Math.abs(robo.Rtrade1.x - robo.location.x) > 0.25)) ||
                    (Math.abs(robo.Rtrade1.y - robo.location.y) > 0.25) ||
                    (Math.abs(robo.Rtrade1.h - robo.location.h) > 0.25))) {
                XYZ(robo.Rtrade1, rtolerance, true, false, 0.15);
            }

            if (loopcount) {
                IntakeExt(robo.intakeleftin, robo.intakerightin);
                IntakePivot(robo.intakepivotback);
                IntakeWrist(robo.intakewrist0);
                loopcount = false;
            }

            if (!robo.clawbeam.getState() && (runtime.milliseconds() >= (drivetimeout - 700))) {
                Claw(robo.clawclosed);

                if (runtime.milliseconds() >= (drivetimeout - 500)) {
                    Intake(robo.intakeopen);

                    if (runtime.milliseconds() >= (drivetimeout - 300)) {
                        ServoArm(robo.armlbackgrab, robo.armrbackgrab);
                        ClawRot(robo.clawrotatebackgrab);

                        if (runtime.milliseconds() >= (drivetimeout - 100)) {
                            Claw(robo.clawautograb);
                        }
                    }
                }
            }
        }

        DriveStop();
        Pos(robo.Rtrade1);

        drivetimeout = runtime.milliseconds() + 500;
        while (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && robo.clawbeam.getState() &&
                (((Math.abs(robo.Rtrade2.x - robo.location.x) > 0.25)) || (Math.abs(robo.Rtrade2.y -
                        robo.location.y) > 0.25) || (Math.abs(robo.Rtrade2.h - robo.location.h) > 0.25))) {
            XYZ(robo.Rtrade2, rtolerance, true, false, 0.15);
        }

        DriveStop();
        Pos(robo.Rtrade2);

        Claw(robo.clawclosed);


        //dostuff
    }

    //region - New/Test?
    public void RightExtraCycles(double timetointake, double cycles, double drivenumber) {
        RightIntake("Left", timetointake);
        RightIntake("Middle", timetointake);
        RightIntake("Right", timetointake);

        while (cyclecount <= cycles) {
            if (cyclecount == 1) {
                tradetimelimit = 1000;
            } else {
                tradetimelimit = 2000;
            }

            RightWall(timetointake, tradetimelimit);
            DeliverRightBar(drivenumber, robo.colortodrop);
            cyclecount = cyclecount + 1;
        }
    }

    public void RightWall(double timetotrade, double tradelimit) {
        IntakeExt(robo.intakeleftin, robo.intakerightin);
        IntakePivot(robo.intakepivotdown);
        IntakeWrist(robo.intakewrist0);

        drivetimeout = timetotrade + tradelimit;
        while (opModeIsActive() && (((Math.abs(robo.righttrade.x - robo.location.x) > 0.25)) || (Math.abs(robo.righttrade.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.righttrade.h - robo.location.h) > 0.25)) && (runtime.milliseconds() < drivetimeout)) {
            XYZ(robo.righttrade, 15, true, false, 0.5);
        }

        DriveStop();
        Pos(robo.righttrade);
////////////////////////////////////////////
        RightReset(robo.armltransfer, robo.armrtransfer, robo.clawrotateup, robo.clawopenfull);

        drivetimeout = timetotrade + 1000;
        while (opModeIsActive() && (((Math.abs(robo.righttrade2.x - robo.location.x) > 0.25)) || (Math.abs(robo.righttrade2.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.righttrade2.h - robo.location.h) > 0.25)) && (runtime.milliseconds() < drivetimeout)) {
            XYZ(robo.righttrade2, 15, true, false, 0.5);
        }

        DriveStop();
        Pos(robo.righttrade2);

        Claw(robo.clawclosed);

        sleep(200);

        ServoArm(robo.armlup, robo.armrup);
        ClawRot(robo.clawflat);
    }

    public void RightIntake(String position, double timetodrive) {
        if (position.matches("Left")) {
            rightcycledrive1 = robo.rightgrabL;
            rightcycledrive2 = robo.rightLdrop;
        } else if (position.matches("Middle")) {
            rightcycledrive1 = robo.rightgrabM;
            rightcycledrive2 = robo.rightMdrop;
        } else if (position.matches("Right")) {
            rightcycledrive1 = robo.rightgrabR;
            //rightcycledrive2 = robo.rightRdrop;
        }

        drivetimeout = timetodrive + 2000;
        while (opModeIsActive() && (((Math.abs(rightcycledrive1.x - robo.location.x) > 0.25)) || (Math.abs(rightcycledrive1.y - robo.location.y) > 0.25) ||
                (Math.abs(rightcycledrive1.h - robo.location.h) > 0.25)) && (runtime.milliseconds() < drivetimeout)) {
            XYZ(rightcycledrive1, 15, true, false, 0.5);
        }

        DriveStop();
        Pos(rightcycledrive1);

        IntakeExt(robo.intakeleftout, robo.intakerightout);
        IntakePivot(robo.intakepivotdown);
        Intake(robo.intakeopen);
        IntakeWrist(robo.intakewrist45);

        sleep(200);

        Intake(robo.intakeclosed);

        sleep(100);

        if (!(position.matches("Right"))) {
            drivetimeout = timetodrive + 1000;
            while (opModeIsActive() && ((Math.abs(rightcycledrive2.h - robo.location.h) > 0.25)) &&
                    (runtime.milliseconds() < drivetimeout)) {
                TURN(rightcycledrive2, robo.otos, true, false, 0.5, 0.2);
            }

            DriveStop();
            Pos(rightcycledrive2);

            IntakeWrist(robo.intakewrist_45);

            sleep(100);

            Intake(robo.intakeopen);
        }
    }
    //endregion

    //region - Old/Redundant?
    public void RightCycle(SparkFunOTOS.Pose2D drivetograbright, double intakewristto, double drivenumber, double liftpos) {
        //region - Right Cycle
        drivetimeout = runtime.milliseconds() + 2000;
        while (opModeIsActive() && (((Math.abs(drivetograbright.x - robo.location.x) > 0.25)) || (Math.abs(drivetograbright.y - robo.location.y) > 0.25) ||
                (Math.abs(drivetograbright.h - robo.location.h) > 0.25)) && (runtime.milliseconds() < drivetimeout)) {
            XYZ(drivetograbright, 25, true, false, 0.5);
        }

        DriveStop();
        Pos(drivetograbright);

        RightReset(robo.armltransfer, robo.armrtransfer, robo.clawrotatetransfer, robo.clawopenfull);

        RightBarcodeGrab(intakewristto);

        RightTradeRoutine(false, true, true);

        DeliverRightBar(drivenumber, robo.colortodrop);
        //endregion
    }

    public void RLastCycle(double drivenumber) {
        drivetimeout = runtime.milliseconds() + 1500;
        while (opModeIsActive() && (((Math.abs(robo.righttrade.x - robo.location.x) > 3)) || (Math.abs(robo.righttrade.y - robo.location.y) > 3) ||
                (Math.abs(robo.righttrade.h - robo.location.h) > 3)) && (runtime.milliseconds() < drivetimeout)) {
            XYZ(robo.righttrade, 15, true, false, 1);
        }
/////////////////////////////////////////////////////
        RightReset(robo.armltransfer, robo.armltransfer, robo.clawrotateup, robo.clawopenfull);

        DriveStop();
        Pos(robo.righttrade);

        RightTradeRoutine(true, true, true);

        DeliverRightBar(drivenumber, robo.colortodrop);
    }

    public void RightBarcodeGrab(double intakewristpos) {
        //region - Rightbarcodegrab
        IntakeExt(robo.intakeleftout, robo.intakerightout);
        IntakePivot(robo.intakepivotdown);
        Intake(robo.intakeopen);
        IntakeWrist(intakewristpos);

        sleep(300);

        Intake(robo.intakeclosed);

        sleep(300);

        IntakeExt(robo.intakeleftin, robo.intakerightin);
        IntakeWrist(robo.intakewrist0);

        drivetimeout = runtime.milliseconds() + 1000; //1500
        while (opModeIsActive() && (((Math.abs(robo.righttrade.x - robo.location.x) > 3)) || (Math.abs(robo.righttrade.y - robo.location.y) > 3) ||
                (Math.abs(robo.righttrade.h - robo.location.h) > 3)) && (runtime.milliseconds() < drivetimeout)) {
            XYZ(robo.righttrade, 3, true, false, 1);
        }

        DriveStop();
        Pos(robo.righttrade);
        //endregion
    }

    public void RightTradeRoutine(boolean righttradefinal, boolean deliverforward, boolean front) {
        if (!righttradefinal) {
            righttradedrive = robo.righttrade2;
        } else {
            righttradedrive = robo.righttrade2final;
        }

        if (front) {
            //////////////////////////////////////////////
            ServoArm(robo.armltransfer, robo.armrtransfer);
            ClawRot(robo.clawrotateup);
        } else {
            ServoArm(robo.armlbackgrab, robo.armrbackgrab);
            ClawRot(robo.clawrotatebackgrab);
        }

        Claw(robo.clawautograb);

        drivetimeout = runtime.milliseconds() + 1000;
        while (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && !robo.clawbeam.getState()) {
            if (opModeIsActive() && (((Math.abs(righttradedrive.x - robo.location.x) > 0.25)) || (Math.abs(righttradedrive.y - robo.location.y) > 0.25) ||
                    (Math.abs(righttradedrive.h - robo.location.h) > 0.25))) {
                XYZ(righttradedrive, 10, true, false, 0.2); //20
            }
        }

        DriveStop();
        Pos(righttradedrive);

        Intake(robo.intakeopen);
        Claw(robo.clawclosed);

        sleep(200); //400

        ServoArm(robo.armlup, robo.armrup);

        sleep(100);

        if (deliverforward) {
            drivetimeout = runtime.milliseconds() + 700;
            while (opModeIsActive() && (Math.abs(robo.rightposttrade.h - robo.location.h) > 0.25) &&
                    (runtime.milliseconds() < drivetimeout)) {
                TURN(robo.rightposttrade, robo.otos, true, false, 1, 0.5);
            }

            DriveStop();
            Pos(robo.rightposttrade);
        }

        drivetimeout = runtime.milliseconds() + 700; //1500
        while (opModeIsActive() && (((Math.abs(robo.rightpredrivebar.x - robo.location.x) > 0.25)) || (Math.abs(robo.rightpredrivebar.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.rightpredrivebar.h - robo.location.h) > 0.25)) && (runtime.milliseconds() < drivetimeout)) {
            XYZ(robo.rightpredrivebar, 5, true, false, 0.3);
        }

        DriveStop();
        Pos(robo.rightpredrivebar);
    }
    //endregion

    public void RightMove(String driveto) {
        /*if (driveto.matches("L")) {
            robo.rdrive1 = robo.rightpremove2;
            robo.rdrive2 = robo.rightpreL;
            robo.rdrive3 = robo.rightmoveL;
        } else if (driveto.matches("M")) {
            robo.rdrive1 = robo.rightpreL;
            robo.rdrive2 = robo.rightpreM;
            robo.rdrive3 = robo.rightmoveM;
        } else if (driveto.matches("R")) {
            robo.rdrive1 = robo.rightpreM;
            robo.rdrive2 = robo.rightpreR;
            robo.rdrive3 = robo.rightmoveR;
        }*/

        drivetimeout = runtime.milliseconds() + 1000;
        while (opModeIsActive() && (((Math.abs(robo.rdrive1.x - robo.location.x) > 0.25)) || (Math.abs(robo.rdrive1.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.rdrive1.h - robo.location.h) > 0.25)) && (runtime.milliseconds() < drivetimeout)) {
            FAST(robo.rdrive1, true, false, 0.5);
        }

        DriveStop();
        Pos(robo.rdrive1);

        drivetimeout = runtime.milliseconds() + 1000;
        while (opModeIsActive() && (((Math.abs(robo.rdrive2.x - robo.location.x) > 0.25)) || (Math.abs(robo.rdrive2.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.rdrive2.h - robo.location.h) > 0.25)) && (runtime.milliseconds() < drivetimeout)) {
            FAST(robo.rdrive2, true, false, 0.5);
        }

        DriveStop();
        Pos(robo.rdrive2);

        drivetimeout = runtime.milliseconds() + 1000;
        while (opModeIsActive() && (((Math.abs(robo.rdrive3.x - robo.location.x) > 0.25)) || (Math.abs(robo.rdrive3.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.rdrive3.h - robo.location.h) > 0.25)) && (runtime.milliseconds() < drivetimeout)) {
            FAST(robo.rdrive3, true, false, 0.5);
        }

        DriveStop();
        Pos(robo.rdrive3);
    }

    public void RightCycleContinuous(boolean fast, boolean lasttrade, double drivecount, boolean front) {
        drivetimeout = runtime.milliseconds() + 1000; //1500
        while (opModeIsActive() && (((Math.abs(robo.righttrade.x - robo.location.x) > 3)) || (Math.abs(robo.righttrade.y - robo.location.y) > 3) ||
                (Math.abs(robo.righttrade.h - robo.location.h) > 3)) && (runtime.milliseconds() < drivetimeout)) {
            if (!fast) {
                XYZ(robo.righttrade, 3, true, false, 1);
            } else {
                FAST(robo.righttrade, true, false, 1);
            }
        }

        DriveStop();
        Pos(robo.righttrade);

        if (front) {
            /////////////////////////////////////////////////////
            RightReset(robo.armltransfer, robo.armrtransfer, robo.clawrotateup, robo.clawautograb);
        } else {
            Claw(robo.clawautograb);
        }

        RightTradeRoutine(lasttrade, false, front);

        DeliverRightBar(drivecount, robo.colortodrop);
    }

    public void RightSubCycle(boolean lastcycle, double drivecount) {
        RSubGrab(false);

        RightReset(robo.armlbackgrab, robo.armrbackgrab, robo.clawrotatebackgrab, robo.clawclosed);

        RightCycleContinuous(true, lastcycle, drivecount, false);
    }

    public void DeliverRightBarOLD(double drive1) {
        //region - RightBarDeliver
        //region - Determine Drive
        if (drive1 == 1) {
            rightdrive1 = robo.Rbar;
            rtolerance = 20;
        } else if (drive1 == 2) {
            rightdrive1 = robo.Rbar2;
            rtolerance = 20;
        } else if (drive1 == 3) {
            rightdrive1 = robo.Rbar3;
            rtolerance = 20;
        } else if (drive1 == 4) {
            rightdrive1 = robo.Rbar4;
            rtolerance = 20;
        }
        //endregion

        liftupto = robo.Lbar2;

        IntakePivot(robo.intakepivotback);
        IntakeExt(robo.intakeleftin, robo.intakerightin);

        ServoArm(robo.armlup, robo.armrup);

        ClawRot(robo.clawflat);

        drivetimeout = runtime.milliseconds() + 1500;
        while ((robo.lift1.getCurrentPosition() < liftupto) || (runtime.milliseconds() < drivetimeout)) {
            if (opModeIsActive() && (((Math.abs(robo.leftdrivebucket.x - robo.location.x) > 0.25)) || (Math.abs(robo.leftdrivebucket.y - robo.location.y) > 0.25) ||
                    (Math.abs(robo.leftdrivebucket.h - robo.location.h) > 0.25))) {
                XYZ(rightdrive1, rtolerance, true, false, 0.15);
            }

            LiftMulti(liftupto, runtime.milliseconds());
        }

        DriveStop();
        LiftStop(robo.lifthigh);
        Pos(rightdrive1);

        Claw(robo.clawopenfull);
        //endregion
    }

    public void RightReset(double armservoposl, double armservoposr, double clawrotpos, double clawpos) {
        Claw(clawpos);
        ServoArm(armservoposl, armservoposr);
        ClawRot(clawrotpos);
        Intake(robo.intakeopen);
        IntakeExt(robo.intakeleftin, robo.intakerightin);
        IntakePivot(robo.intakepivotback);
        Liftdown(runtime.milliseconds());

        LiftStop(robo.liftdown);
    }

    public void ParkRight() {
        //region - RightPark
        drivetimeout = runtime.milliseconds() + 2000;
        while (opModeIsActive() && (((Math.abs(robo.rightpark.x - robo.location.x) > 0.25)) || (Math.abs(robo.rightpark.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.rightpark.h - robo.location.h) > 0.25)) && (runtime.milliseconds() < drivetimeout)) {
            XYZ(robo.rightpark, 10, true, false, 0.5);
        }

        DriveStop();
        Pos(robo.rightpark);

        RightReset(robo.armltransfer, robo.armrtransfer, robo.clawrotatetransfer, robo.clawopenfull);
        //endregion
    }

    public void RSubGrab(boolean yellow) {
        IntakeExt(robo.intakeleftin, robo.intakerightin);
        IntakePivot(robo.intakepivotback);
        Intake(robo.intakeopen);
        IntakeWrist(robo.intakewrist0);

        drivetimeout = runtime.milliseconds() + 2000;
        while (opModeIsActive() && (((Math.abs(robo.park.x - robo.location.x) > 0.25)) || (Math.abs(robo.park.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.park.h - robo.location.h) > 0.25)) && (runtime.milliseconds() < drivetimeout)) {
            FAST(robo.park, true, false, 0.5); //XYZ?
        }

        DriveStop();
        Pos(robo.park);

        IntakePivot(robo.intakepivotscan);
        ClawRot(robo.clawrotatescan);

        if (yellow) {
            YellowGrab(3);
        } else {
            //specimen grab here
        }
    }
    //endregion
}
