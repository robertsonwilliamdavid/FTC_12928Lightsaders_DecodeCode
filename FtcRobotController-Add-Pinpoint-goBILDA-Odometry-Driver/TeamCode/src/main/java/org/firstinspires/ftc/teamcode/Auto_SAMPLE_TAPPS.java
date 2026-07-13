package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.opencv.ColorRange;
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

@Autonomous(name = "Auto_SAMPLE_TAPPS", group = "Linear Opmode")
public class Auto_SAMPLE_TAPPS extends LinearOpMode {

    DeepRoboConstants robo = new DeepRoboConstants();
    OpenCvCamera webcam;
    SampleAlignmentPipeline pipeline;

    //region - Variables
    private ElapsedTime runtime = new ElapsedTime();

    double drivetimeout = 0;
    double tradetimelimit = 0;
    double extradrivetime = 0;
    boolean stopdrive = false;
    boolean resetotos = false;
    boolean resetready = false;

    boolean lift = false;
    boolean transfer = false;

    double grabtime = 0;
    double closetime = 0;
    boolean intake = false;
    boolean clawgrab = false;
    boolean grabbing = false;

    double changedrive = 0;
    double grab1zone = 0;
    double grab2zone = 0;
    double grab1add = 0;
    double grab2add = 0;

    //region - New
    static Point samplecenter = new Point();
    static double sampleangle = 0;
    static double grabangle = 0;

    static Point testpoint = new Point(320, 240);

    static double sampledist = 0;
    static double servoextend = 0;
    static double disttostrafe = 0;
    static double bottomy = 0;

    static boolean scannow = false;
    int colorcount = 0;
    boolean reset = false;

    Point lastcenter = new Point(0, 0);
    double lastangle = 0;
    double scantime = 0;
    //endregion

    double subtime = 0;
    boolean driving = false;
    boolean needtodrive = false;
    boolean drop = false;
    double droptime = 0;
    boolean stop = false;
    double stoptime = 0;
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
            if (gamepad1.left_trigger > 0.5) {
                robo.startpos = "Left";
            } else if (gamepad1.right_trigger > 0.5) {
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

            if (robo.startpos.matches("Left")) {
                if (gamepad1.left_bumper) {
                    grab1zone = changedrive;
                } else if (gamepad1.right_bumper) {
                    grab2zone = changedrive;
                }

                if (gamepad1.dpad_up) {
                    changedrive = 0;
                } else if (gamepad1.dpad_left) {
                    changedrive = 1;
                } else if (gamepad1.dpad_down) {
                    changedrive = 2;
                } else if (gamepad1.dpad_right) {
                    changedrive = 3;
                }

                grab1add = grab1zone * 5;
                grab2add = grab2zone * 5;
            }

            telemetry.addData("Press Left for Left", "");
            telemetry.addData("Press Right for Right", "");
            telemetry.addData("Press Colors for Cycle", "");

            if (robo.startpos.matches("Left")) {
                telemetry.addData("Dpad for Sub Pos", "");
                telemetry.addData("Bumpers to Set Drive", "");
            }

            telemetry.addData("Press A to Continue", "");
            telemetry.addData("Start Position: ", robo.startpos);
            telemetry.addData("Color to Grab: ", robo.GrabColor);
            telemetry.addData("Team Color Is: ", robo.teamcolor);

            if (robo.startpos.matches("Left")) {
                telemetry.addData("Drive Zone: ", changedrive);
                telemetry.addData("Grab1: ", grab1zone);
                telemetry.addData("Grab2: ", grab2zone);
                telemetry.addData("Grab1add: ", grab1add);
                telemetry.addData("Grab2add: ", grab2add);
            }

            telemetry.update();
        }
        //endregion

        //region - Camera Init
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        webcam = OpenCvCameraFactory.getInstance().createWebcam(camera, cameraMonitorViewId);
        pipeline = new SampleAlignmentPipeline();

        // Specify the image processing pipeline we wish to invoke upon receipt of a frame from the camera.
        // Note that switching pipelines on-the-fly (while a streaming session is in flight) *IS* supported.
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });
        //endregion

        robo.clawrotate.setPosition(robo.clawrotateinit);
        robo.arml.setPosition(robo.armlinit);
        robo.armr.setPosition(robo.armrinit);
        robo.intakepivot.setPosition(robo.intakepivotback);

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

        ServoArm(robo.armlup, robo.armrup);
        ClawRot(robo.clawrotatedeliverb);

        drivetimeout = runtime.milliseconds() + 1250; //1300
        resetready = true;
        while (((robo.lift1.getCurrentPosition() < robo.lifthigh) || (runtime.milliseconds() < (drivetimeout)))
                && opModeIsActive()) { //below was -300 ms
            if ((runtime.milliseconds() < (drivetimeout)) && (((Math.abs(robo.del.x - robo.location.x) >
                    0.25)) || (Math.abs(robo.del.y - robo.location.y) > 0.25) || (Math.abs(robo.del.h -
                    robo.location.h) > 0.25))) {
                XYZ(robo.del, 8, true, false, 0.2); //0.3
            } else if (resetready) {
                DriveHold(false);
                DriveStop();
                resetotos = true;
                resetready = false;
            }

            LiftMulti(robo.lifthigh, (drivetimeout - 1300));
        }

        DriveHold(false);
        DriveStop();
        LiftStop(robo.lifthigh);
        Pos(robo.del);

        DeliverLeft(true, 1);

        //region - Cycles
        LeftBarcodeGrab(robo.fgr, 0.2);

        DeliverLeft(true, 2);

        LeftBarcodeGrab(robo.fgm, 0.3); //0.4 //0.5

        DeliverLeft(true, 3);

        LeftBarcodeGrab(robo.fgl, 0.7);

        DeliverLeft(false, 0);
        //endregion

        //region - Submersible
        YSub(grab1add, robo.tosub1);

        while (reset) {
            if (runtime.milliseconds() > (subtime + 700)) {
                subtime = runtime.milliseconds();
                YSub(grab1add, robo.tosub1);
            }
        }

        DeliverLeft(false, 0);

        YSub(grab2add, robo.tosub1); //2

        while (reset) {
            if (runtime.milliseconds() > (subtime + 700)) {
                subtime = runtime.milliseconds();
                YSub(grab2add, robo.tosub1); //2
            }
        }

        DeliverLeft(false, 0);
        //endregion

        ParkLeft();
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
        robo.drivetarget = driveto;

        Log.d("PositionCheck", "Target = " + driveto.x + ", " + driveto.y + ", " + driveto.h);

        robo.ytarget = robo.drivetarget.y;
        robo.xtarget = robo.drivetarget.x;
        robo.ztarget = robo.drivetarget.h;

        robo.otosloc = robo.otos.getPosition();
        robo.roboorigin = new SparkFunOTOS.Pose2D(0, 0, 0);

        robo.location.x = robo.otosloc.x;
        robo.location.y = robo.otosloc.y;
        robo.location.h = robo.otosloc.h;

        Log.d("PositionCheck", "X is: " + robo.location.x);
        Log.d("PositionCheck", "Y is: " + robo.location.y);
        Log.d("PositionCheck", "Z is: " + robo.location.h);

        if (((Math.abs(robo.ytarget - robo.location.y)) > 0.25) || ((Math.abs(robo.xtarget - robo.location.x)) > 0.25) ||
                ((Math.abs(robo.ztarget - robo.location.h)) > 1)) {
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

            robo.flpower = Range.clip(robo.ypower + robo.xpower - robo.zpower, -1, 1);
            robo.frpower = Range.clip(robo.ypower - robo.xpower + robo.zpower, -1, 1);
            robo.blpower = Range.clip(robo.ypower - robo.xpower - robo.zpower, -1, 1);
            robo.brpower = Range.clip(robo.ypower + robo.xpower + robo.zpower, -1, 1);
            //endregion

            //region - Apply Motor Power
            robo.fl.setPower(robo.flpower);
            robo.fr.setPower(robo.frpower);
            robo.bl.setPower(robo.blpower);
            robo.br.setPower(robo.brpower);
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

        sleep(10); //50

        DriveStop();
    }

    public void Adjust(SparkFunOTOS.Pose2D driveto, double tolerance) {
        robo.drivetarget = driveto;

        Log.d("PositionCheck", "XTarget: " + robo.drivetarget.x);
        Log.d("PositionCheck", "YTarget: " + robo.drivetarget.y);

        robo.xtarget = robo.drivetarget.x;
        robo.ytarget = robo.drivetarget.y;

        robo.otosloc = robo.otos.getPosition();
        robo.roboorigin = new SparkFunOTOS.Pose2D(0, 0, 0);

        robo.location.y = robo.otosloc.y;
        robo.location.x = robo.otosloc.x;
        robo.location.h = robo.otosloc.h;

        Log.d("PositionCheck", "X is: " + robo.location.x);
        Log.d("PositionCheck", "Y is: " + robo.location.y);

        if (((Math.abs(robo.xtarget - robo.location.x)) > 0.25) || ((Math.abs(robo.ytarget - robo.location.y)) > 0.25)) {
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

            if ((Math.abs(robo.ytarget - robo.location.y)) > tolerance) { //15
                if ((robo.ytarget - robo.location.y) > 0) {
                    robo.yerror = 1;
                } else if ((robo.ytarget - robo.location.y) < 0) {
                    robo.yerror = -1;
                }
            } else {
                robo.yerror = Range.clip(((robo.ytarget - robo.location.y) / tolerance), -1, 1); //15
            }

            robo.xpower = robo.xerror;
            robo.ypower = robo.yerror;

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
            //endregion

            //flip x and y powers when cross 90 threshold to correct
            if (Math.abs(robo.location.h) > 90) {
                robo.xpower = -robo.xpower;
                robo.ypower = -robo.ypower;
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

            robo.flpower = Range.clip(robo.ypower + robo.xpower, -0.5, 0.5);
            robo.frpower = Range.clip(robo.ypower - robo.xpower, -0.5, 0.5);
            robo.blpower = Range.clip(robo.ypower - robo.xpower, -0.5, 0.5);
            robo.brpower = Range.clip(robo.ypower + robo.xpower, -0.5, 0.5);

            if (Math.abs(robo.flpower) < 0.35) {
                if (robo.flpower > 0) {
                    robo.flpower = 0.35;
                } else {
                    robo.flpower = -0.35;
                }
            }

            if (Math.abs(robo.frpower) < 0.35) {
                if (robo.frpower > 0) {
                    robo.frpower = 0.35;
                } else {
                    robo.frpower = -0.35;
                }
            }

            if (Math.abs(robo.blpower) < 0.35) {
                if (robo.blpower > 0) {
                    robo.blpower = 0.35;
                } else {
                    robo.blpower = -0.35;
                }
            }

            if (Math.abs(robo.brpower) < 0.35) {
                if (robo.brpower > 0) {
                    robo.brpower = 0.35;
                } else {
                    robo.brpower = -0.35;
                }
            }
            //endregion

            //region - Apply Motor Power
            robo.fl.setPower(robo.flpower + 0.02);
            robo.fr.setPower(robo.frpower + 0.02);
            robo.bl.setPower(robo.blpower + 0.02);
            robo.br.setPower(robo.brpower + 0.02);
            //endregion
        }
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
        if ((robo.lift1.getCurrentPosition() < liftupto) && (runtime.milliseconds() < (uptime + 2000))) {
            robo.lift1.setPower(1);
            robo.lift2.setPower(1);
            robo.lift3.setPower(1);

            Log.d("LIFT", "Lift Pos: " + robo.lift1.getCurrentPosition());
        } else {
            LiftStop(liftupto);
        }
    }

    public void Liftdown(double downtime) {
        while ((robo.lift1.getCurrentPosition() > robo.liftdown) && (robo.liftMag.getState()) && (downtime < (downtime + 1200))) { //1000
            robo.lift1.setPower(-1);
            robo.lift2.setPower(-1);
            robo.lift3.setPower(-1);
        }

        LiftStop(robo.liftdown);
    }

    public void LiftdownMulti(double downtime) {
        if ((robo.lift1.getCurrentPosition() > robo.liftdown) && (robo.liftMag.getState()) && (runtime.milliseconds() <
                (downtime + 1200))) {
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
    public void DeliverLeft(boolean floor, double grab) {
        //region - CycleLeft
        DriveStop();

        if (floor) {
            IntakeExt(robo.intakeleftout, robo.intakerightout);
            IntakePivot(robo.intakepivotdown);
            Intake(robo.intakeopen);

            if (grab == 1) {
                IntakeWrist(robo.intakewrist25);
            } else if (grab == 2) {
                IntakeWrist(robo.intakewrist0);
            } else {
                IntakeWrist(robo.intakewrist_45);
            }
        }

        ServoArm(robo.armldeliver, robo.armrdeliver);

        sleep(75); //60 //45

        Claw(robo.clawopenfull);

        sleep(75); //100

        ClawRot(robo.clawrotatetransfer);
        //endregion
    }

    public void LeftBarcodeGrab(SparkFunOTOS.Pose2D drivetoposleft, double turnpower) {
        //region - Drivetoposleft
        drivetimeout = runtime.milliseconds() + 1000;
        intake = true;
        stopdrive = true;
        extradrivetime = 700;
        grabbing = true;
        robo.colorseen = "None";
        while ((((robo.lift1.getCurrentPosition() > robo.liftdown) && (runtime.milliseconds() < (drivetimeout))) || grabbing) && opModeIsActive()) {
            if (runtime.milliseconds() > (drivetimeout - 600)) {
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

            Log.d("Color", "Colorseen: " + robo.colorseen);
            Log.d("Test", "Intake: " + intake);
            Log.d("Test", "Grabbing: " + grabbing);

            if ((runtime.milliseconds() < (drivetimeout)) && ((!robo.colorseen.matches("Yellow")) &&
                    !robo.colorseen.matches(robo.teamcolor)) && intake && grabbing && (((Math.abs(drivetoposleft.x -
                    robo.location.x) > 0.25)) || (Math.abs(drivetoposleft.y - robo.location.y) > 0.25) || (Math.abs(
                    drivetoposleft.h - robo.location.h) > 0.25))) {
                Log.d("Test", "In Grab Drive 1 pt1");

                XYZ(drivetoposleft, 16, true, false, turnpower); //15 //0.7

                Log.d("Test", "In Grab Drive 1 pt2");
            } else if ((!robo.colorseen.matches("Yellow") && !robo.colorseen.matches(robo.teamcolor)) && grabbing) {
                robo.fl.setPower(-0.35);
                robo.fr.setPower(0.45);
                robo.bl.setPower(0.4);
                robo.br.setPower(-0.4);
            }/* else if (stopdrive) {
                DriveHold(false);
                DriveStop();
                stopdrive = false;
            }*/

            if (intake && (robo.colorseen.matches("Yellow") || (robo.colorseen.matches(robo.teamcolor))) && (runtime.milliseconds() > (drivetimeout - 600))) {
                DriveHold(true);
                DriveStop();
                stopdrive = false;

                Intake(robo.intakeclosed);

                closetime = runtime.milliseconds() + 250; //300
                intake = false;
            }

            if ((!stopdrive) && (runtime.milliseconds() > closetime)) {
                IntakePivot(robo.intakepivotback);
                IntakeExt(robo.intakeleftin, robo.intakerightin);
                IntakeWrist(robo.intakewrist0);
                grabbing = false;
            }

            if (runtime.milliseconds() > (drivetimeout - 900)) {
                LiftdownMulti(drivetimeout - 1000);
            }

            if (runtime.milliseconds() > (drivetimeout - 950)) {
                ServoArm(robo.armlup, robo.armrup);
            }
        }

        DriveStop();
        LiftStop(robo.liftdown);
        Pos(drivetoposleft);

        Claw(robo.clawautograb);
        ServoArm(robo.armltransfer, robo.armrtransfer);

        transfer = false;
        lift = false;
        clawgrab = false;
        robo.loopcheck = true;
        drivetimeout = runtime.milliseconds() + 2000; //1300
        while ((robo.lift1.getCurrentPosition() < robo.lifthigh) && (runtime.milliseconds() < drivetimeout)) {
            if (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && (((Math.abs(robo.del2.x -
                    robo.location.x) > 0.25)) || (Math.abs(robo.del2.y - robo.location.y) > 0.25) ||
                    (Math.abs(robo.del2.h - robo.location.h) > 0.25))) {
                XYZ(robo.del2, 15, true, false, 0.8); //FAST?
            } else {
                DriveStop();
            }

            if (lift) {
                LiftMulti(robo.lifthigh, (drivetimeout - 2000));
            }

            if (!robo.clawbeam.getState() && robo.loopcheck && (runtime.milliseconds() > (drivetimeout - 1800))) { //1800
                grabtime = runtime.milliseconds();
                robo.loopcheck = false;
                transfer = true;
            }

            if (transfer) {
                //if (runtime.milliseconds() >= (grabtime + 75)) { //150
                Claw(robo.clawclosed);

                if (runtime.milliseconds() >= (grabtime + 150)) { ///250 //275 //400
                    Intake(robo.intakeopen);

                    //if (runtime.milliseconds() >= (grabtime + 400)) { //550
                    ServoArm(robo.armlup, robo.armrup);
                    ClawRot(robo.clawrotatedeliverb);
                    lift = true;
                    transfer = false;
                    //}
                }
                //}
            }
        }

        DriveStop();
        LiftStop(robo.lifthigh);
        Pos(robo.del2);
        lift = false;
        //endregion
    }

    public void ParkLeft() {
        //region - LeftPark
        ServoArm(robo.armlup, robo.armrup);
        ClawRot(robo.clawflat);
        Claw(robo.clawopenfull);
        IntakeWrist(robo.intakewrist0);
        IntakePivot(robo.intakepivotback);
        IntakeExt(robo.intakeleftin, robo.intakerightin);
        Intake(robo.intakeopen);

        sleep(200); //150 //250

        drivetimeout = runtime.milliseconds() + 1000; //1400
        while ((robo.lift1.getCurrentPosition() > robo.liftdown) && (runtime.milliseconds() < drivetimeout)) {
            if (opModeIsActive() && (((Math.abs(robo.tosub2.x - robo.location.x) > 0.25)) || (Math.abs(robo.tosub2.y - robo.location.y) > 0.25) ||
                    (Math.abs(robo.tosub2.h - robo.location.h) > 0.25))) {
                XYZ(robo.tosub2, 10, true, false, 0.8); //0.2
            }

            LiftdownMulti(drivetimeout - 1000);
        }

        DriveStop();
        LiftStop(robo.liftdown);
        Pos(robo.tosub2);

        ServoArm(robo.armlup, robo.armrup);
        ClawRot(robo.clawflat);

        drivetimeout = runtime.milliseconds() + 1000; //700?
        while ((runtime.milliseconds() < drivetimeout) && opModeIsActive() && (((Math.abs(robo.END.x -
                robo.location.x) > 0.25)) || (Math.abs(robo.END.y - robo.location.y) > 0.25) ||
                (Math.abs(robo.END.h - robo.location.h) > 0.25))) {
            XYZ(robo.END, 7, true, false, 0.8); //1
        }

        DriveStop();
        Pos(robo.END);

        ServoArm(robo.armlpark, robo.armrpark);

        sleep(200);
        //endregion
    }

    public void YSub(double change, SparkFunOTOS.Pose2D presub) {
        Log.d("Checkpoint", "In YSub()");

        if (!reset) {
            Log.d("Checkpoint", "In 1st Loop");

            ServoArm(robo.armltransfer, robo.armrtransfer);
            ClawRot(robo.clawrotatescan);
            Claw(robo.clawopenfull);
            IntakeWrist(robo.intakewrist0);
            IntakePivot(robo.intakepivotscan);
            IntakeExt(robo.intakeleftin, robo.intakerightin);
            Intake(robo.intakeopen);

            sleep(200); //150 //250

            Liftdown(runtime.milliseconds());

            drivetimeout = runtime.milliseconds() + 1000; //1200
            while (/*(robo.lift1.getCurrentPosition() > robo.liftdown) || */(runtime.milliseconds() < drivetimeout)) {
                if (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && (((Math.abs(presub.x -
                        robo.location.x) > 0.25)) || (Math.abs(presub.y - robo.location.y) > 0.25) ||
                        (Math.abs(presub.h - robo.location.h) > 0.25))) {
                    XYZ(presub, 10, true, false, 0.8); //1, 0.5
                }

                //LiftdownMulti(drivetimeout - 1200);
            }

            DriveStop();
            //LiftStop(robo.liftdown);
            Pos(presub);

            Log.d("Checkpoint", "Time Taken1: " + (drivetimeout - runtime.milliseconds()));
            Log.d("Checkpoint", "Post Drive 1");

            robo.GRAB = robo.PARK;
            robo.GRAB.y = robo.PARK.y + change;

            drivetimeout = runtime.milliseconds() + 700; //800
            while ((runtime.milliseconds() < drivetimeout) && opModeIsActive() && (((Math.abs(robo.GRAB.x -
                    robo.location.x) > 0.25)) || (Math.abs(robo.GRAB.y - robo.location.y) > 0.25) ||
                    (Math.abs(robo.GRAB.h - robo.location.h) > 0.25))) {
                XYZ(robo.GRAB, 7, true, false, 0.8); //1, 0.5
            }

            Log.d("Checkpoint", "Time Taken2: " + (drivetimeout - runtime.milliseconds()));

            DriveStop();
            Pos(robo.GRAB);
        }

        Log.d("Checkpoint", "Post To Sub");

        if (robo.pin0.getState() && robo.pin1.getState()) {
            robo.colorseen = "Yellow";
        } else if (robo.pin0.getState()) {
            robo.colorseen = "Blue";
        } else if (robo.pin1.getState()) {
            robo.colorseen = "Red";
        } else {
            robo.colorseen = "None";
        }

        String drop;

        if (robo.teamcolor.matches("Red")) {
            drop = "Blue";
        } else {
            drop = "Red";
        }

        if (robo.colorseen.matches(drop)) {
            robo.intakepivot.setPosition(robo.intakepivotmid);
            robo.intakeclaw.setPosition(robo.intakeopen);

            sleep(100);

            robo.intakepivot.setPosition(robo.intakepivotscan);
        }

        if (!robo.colorseen.matches("Yellow") && !robo.colorseen.matches(robo.teamcolor)) {
            drivetimeout = runtime.milliseconds() + 250;
            while ((runtime.milliseconds() < drivetimeout) && opModeIsActive() && reset) {
                robo.fl.setPower(-0.8);
                robo.fr.setPower(0.8);
                robo.bl.setPower(0.8);
                robo.br.setPower(-0.8);
            }

            Log.d("Checkpoint", "Post ReAdjust If Needed");

            DriveHold(true);
            DriveStop();
            //}

            if (!reset) {
                sleep(350);
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

            if (!robo.colorseen.matches("Yellow") && !robo.colorseen.matches(robo.teamcolor)) {
                Log.d("Checkpoint", "Scanning");

                //Point origin = new Point(0, 0);

                scantime = runtime.milliseconds() + 300; //500 //200
                while (((lastcenter == pipeline.getCenter()) && (pipeline.getCenter().y > 0)) || ((runtime.milliseconds() <
                        scantime)/* && ((runtime.milliseconds() - scantime) < 800)*/) && !robo.colorseen.matches("Yellow") && !robo.colorseen.matches(robo.teamcolor)) {
                    scannow = true;

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

                //sleep(500); //800

                scannow = false;

                Log.d("Checkpoint", "Stop Scan");
                Log.d("Camera", "CENTER, GRAB ANGLE: (" + pipeline.getCenter().x + ", " + pipeline.getCenter().y
                        + "), " + pipeline.getGrabAngle());

                if (robo.pin0.getState() && robo.pin1.getState()) {
                    robo.colorseen = "Yellow";
                } else if (robo.pin0.getState()) {
                    robo.colorseen = "Blue";
                } else if (robo.pin1.getState()) {
                    robo.colorseen = "Red";
                } else {
                    robo.colorseen = "None";
                }

                if (!robo.colorseen.matches("Yellow") && !robo.colorseen.matches(robo.teamcolor)) {
                    robo.location = robo.otos.getPosition();
                    SparkFunOTOS.Pose2D grabpos = new SparkFunOTOS.Pose2D(robo.location.x, (robo.location.y +
                            pipeline.getStrafeDist()), robo.location.h);

                    Log.d("Checkpoint", "Position of Robot: " + robo.location.x + ", " + robo.location.y + ", " + robo.location.h);
                    Log.d("Checkpoint", "Position of Sample: " + grabpos.x + ", " + grabpos.y + ", " + grabpos.h);

                    double extratime = Math.abs(100 * pipeline.getStrafeDist());

                    if (extratime < 150) {
                        extratime = 150;
                    } else if (extratime > 325) { //400
                        extratime = 325;
                    }

                    drivetimeout = runtime.milliseconds() + extratime; //250, 500
                    while ((runtime.milliseconds() < drivetimeout) && opModeIsActive() && (Math.abs(grabpos.y -
                            robo.location.y) > 0.25)) {
                        Adjust(grabpos, 5); //4
                    }

                    DriveStop();
                    Pos(grabpos);

                    lastcenter = pipeline.getCenter();
                    lastangle = pipeline.getGrabAngle();

                    Log.d("Checkpoint", "Post Sample Adjust");

                    robo.intakepivot.setPosition(robo.intakepivotdown);
                    robo.intakeclaw.setPosition(robo.intakeopen);

                    sleep(50);

                    robo.inExtL.setPosition(Range.clip(robo.intakeleftin - pipeline.getServoExt(), robo.intakeleftout, robo.intakeleftin));
                    robo.inExtR.setPosition(Range.clip(robo.intakerightin + pipeline.getServoExt(), robo.intakerightin, robo.intakerightout));

                    sleep(50);

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

                    Log.d("Checkpoint", "Post Extend");

                    sleep(200); //150?

                    Intake(robo.intakeclosed);

                    sleep(300); //250

                    IntakePivot(robo.intakepivotback);
                    IntakeWrist(robo.intakewrist0);
                    IntakeExt(robo.intakeleftin, robo.intakerightin);

                    sleep(100); //250

                    Log.d("Checkpoint", "Post Grab");
                }
            }
        }

        while (colorcount < 10) { //8
            if (robo.pin0.getState() && robo.pin1.getState()) {
                robo.colorseen = "Yellow";
            } else if (robo.pin0.getState()) {
                robo.colorseen = "Blue";
            } else if (robo.pin1.getState()) {
                robo.colorseen = "Red";
            } else {
                robo.colorseen = "None";
            }

            colorcount = colorcount + 1;
        }

        Log.d("Checkpoint", "Color Seen: " + robo.colorseen);

        if (!robo.colorseen.matches("Yellow") && !robo.colorseen.matches(robo.teamcolor)) {
            reset = true;
            Log.d("Checkpoint", "Retry");
        } else {
            ClawRot(robo.clawrotatetransfer);
            Claw(robo.clawautograb);

            reset = false;
            transfer = false;
            robo.loopcheck = true;
            drivetimeout = runtime.milliseconds() + 1100; //1200
            while ((runtime.milliseconds() < drivetimeout) && opModeIsActive() && (((Math.abs(robo.tobucket.x -
                    robo.location.x) > 0.25)) || (Math.abs(robo.tobucket.y - robo.location.y) > 0.25) ||
                    (Math.abs(robo.tobucket.h - robo.location.h) > 0.25))) {
                XYZ(robo.tobucket, 1, true, false, 1); //0.5

                if (!robo.clawbeam.getState() && robo.loopcheck && (runtime.milliseconds() > (drivetimeout - 1100))) {
                    grabtime = runtime.milliseconds();
                    robo.loopcheck = false;
                    transfer = true;
                    Claw(robo.clawclosed);
                }

                if (transfer) {
                    if (runtime.milliseconds() >= (grabtime + 200)) { //150 //275
                        Intake(robo.intakeopen);

                        //if (runtime.milliseconds() >= (grabtime + 325)) { //400
                        ServoArm(robo.armlup, robo.armrup);
                        ClawRot(robo.clawrotatedeliverb);
                        transfer = false;

                        Log.d("Checkpoint", "Transfer Complete");
                        //}
                    }
                }

                Log.d("Checkpoint", "Driving to Bucket1");
            }

            DriveStop();
            Pos(robo.tobucket);

            Log.d("Checkpoint", "At Midpoint");

            clawgrab = false;
            drivetimeout = runtime.milliseconds() + 1700; //1500
            while ((robo.lift1.getCurrentPosition() < robo.lifthigh) && (runtime.milliseconds() < drivetimeout)) {
                if (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && (((Math.abs(robo.del3.x -
                        robo.location.x) > 0.25)) || (Math.abs(robo.del3.y - robo.location.y) > 0.25) ||
                        (Math.abs(robo.del3.h - robo.location.h) > 0.25))) {
                    XYZ(robo.del3, 25, true, false, 0.1); //15, 0.5
                } else {
                    DriveStop();
                }

                LiftMulti(robo.lifthigh, (drivetimeout - 1700));
            }

            DriveStop();
            LiftStop(robo.lifthigh);
            Pos(robo.del3);
            lift = false;

            Log.d("Checkpoint", "At Bucket");
        }
    }
    //endregion

    //region - Pipeline
    static class SampleAlignmentPipeline extends OpenCvPipeline {
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

        Scalar green = new Scalar(120, 255, 180);
        Scalar white = new Scalar(255, 255, 255);

        @Override
        public void init(Mat firstFrame) {
        }

        public void houghPolar(Mat input) {
            Mat blur = new Mat();
            Imgproc.bilateralFilter(input, blur, 3, 59, 117); //81, 136

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

            for (int x = 0; x < lines.rows(); x++) {
                double[] l = lines.get(x, 0);
                double slope = (l[3] - l[1]) / (l[2] - l[0]);
                double intercept = l[1] - (slope * l[0]);
                double length = Math.sqrt(Math.pow((l[2] - l[0]), 2) + Math.pow((l[3] - l[1]), 2));

                LineData linedata = new LineData(new Point(l[0], l[1]), new Point(l[2], l[3]), slope, intercept, length);
                AllLines.add(linedata);
            }

            ArrayList<PLineData> parallel = new ArrayList<>();

            for (int z = 0; z < (AllLines.size() - 1); z++) {
                for (int w = z + 1; w < AllLines.size(); w++) {
                    double pt1check = Math.sqrt(Math.abs(Math.pow((AllLines.get(w).pt1.x - AllLines.get(z).pt1.x), 2) +
                            Math.pow((AllLines.get(w).pt1.y - AllLines.get(z).pt1.y), 2)));
                    double pt2check = Math.sqrt(Math.abs(Math.pow((AllLines.get(w).pt2.x - AllLines.get(z).pt2.x), 2) +
                            Math.pow((AllLines.get(w).pt2.y - AllLines.get(z).pt2.y), 2)));

                    if (Math.abs((AllLines.get(z).slope - AllLines.get(w).slope) / (AllLines.get(w).slope + 1e-6)) < 0.99) {
                        if ((pt1check > 58) && (pt2check > 58) && (pt1check < 110) && (pt2check < 110)) {
                            PLineData parallellines = new PLineData(AllLines.get(z).pt1, AllLines.get(z).pt2,
                                    AllLines.get(w).pt1, AllLines.get(w).pt2);
                            parallel.add(parallellines);

                            double centerx = (Math.min(Math.min(parallellines.pt1.x, parallellines.pt2.x), Math.min(
                                    parallellines.pt3.x, parallellines.pt4.x)) + (Math.max(Math.max(parallellines.pt1.x,
                                    parallellines.pt2.x), Math.max(parallellines.pt3.x, parallellines.pt4.x)))) / 2;
                            double centery = (Math.min(Math.min(parallellines.pt1.y, parallellines.pt2.y), Math.min(
                                    parallellines.pt3.y, parallellines.pt4.y)) + (Math.max(Math.max(parallellines.pt1.y,
                                    parallellines.pt2.y), Math.max(parallellines.pt3.y, parallellines.pt4.y)))) / 2;

                            Point center = new Point(centerx, centery);

                            double h = (Math.sqrt(Math.abs(Math.pow((parallellines.pt3.x - parallellines.pt1.x), 2) +
                                    Math.pow((parallellines.pt3.y - parallellines.pt1.y), 2))) + Math.sqrt(Math.abs(
                                    Math.pow((parallellines.pt4.x - parallellines.pt2.x), 2) +
                                            Math.pow((parallellines.pt4.y - parallellines.pt2.y), 2)))) / 2;
                            double area = (0.5) * (AllLines.get(z).length + AllLines.get(w).length) * (h);

                            double otherslope1 = (AllLines.get(w).pt1.y - AllLines.get(z).pt1.y) /
                                    (AllLines.get(w).pt1.x - AllLines.get(z).pt1.x);
                            double otherslope2 = (AllLines.get(w).pt2.y - AllLines.get(z).pt2.y) /
                                    (AllLines.get(w).pt2.x - AllLines.get(z).pt2.x);
                            double theta1 = (Math.atan(Math.abs((AllLines.get(z).slope - otherslope1) /
                                    (1 + (AllLines.get(z).slope * otherslope1))))) * (180 / Math.PI);
                            double theta2 = (Math.atan(Math.abs((AllLines.get(w).slope - otherslope2) /
                                    (1 + (AllLines.get(w).slope * otherslope2))))) * (180 / Math.PI);
                            double theta3 = (Math.atan(Math.abs((AllLines.get(z).slope - otherslope2) /
                                    (1 + (AllLines.get(z).slope * otherslope1))))) * (180 / Math.PI);
                            double theta4 = (Math.atan(Math.abs((AllLines.get(w).slope - otherslope1) /
                                    (1 + (AllLines.get(w).slope * otherslope2))))) * (180 / Math.PI);

                            double[] pixel = hsv.get((int) center.y, (int) center.x);
                            boolean centeryellow = false;

                            if ((8 < pixel[0]) && (pixel[0] < 40) && (20 < pixel[1]) && (pixel[1] < 255) && (170 < pixel[2]) && (pixel[2] < 255)) {
                                centeryellow = true;
                            }

                            //min area = 4750
                            if ((area > 7000) && (area < 16000) && (((theta1 > 60) && (theta2 > 60)) || ((theta3 > 60) && (theta4 > 60))) &&
                                    centeryellow && (center.y > 50) && (center.y < 380) && (center.x > 100) && (center.x < 540)) { //100 for y
                                Imgproc.line(dst, parallellines.pt1, parallellines.pt2, green, 3, Imgproc.LINE_AA);
                                Imgproc.line(dst, parallellines.pt3, parallellines.pt4, green, 3, Imgproc.LINE_AA);
                                Imgproc.line(dst, parallellines.pt1, parallellines.pt3, green, 3, Imgproc.LINE_AA);
                                Imgproc.line(dst, parallellines.pt2, parallellines.pt4, green, 3, Imgproc.LINE_AA);

                                Imgproc.circle(dst, center, 2, green, 5);

                                samplecenter = center;

                                double dy = parallellines.pt1.y - parallellines.pt2.y;
                                double dx = parallellines.pt1.x - parallellines.pt2.x;
                                double angle = (Math.atan2(dy, dx) * (180 / Math.PI)) - 90;

                                if (angle < 0) {
                                    angle = angle + 180;
                                }

                                sampleangle = angle;

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

            lines.release();
        }

        @Override
        public Mat processFrame(Mat input) {
            if (scannow) {
                Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV, 4);

                //yellow
                Core.inRange(hsv, new Scalar(8, 20, 170), new Scalar(40, 255, 255), colorG);

                result = input;

                houghPolar(colorG);

                Imgproc.circle(result, samplecenter, 2, green, 5);
                Imgproc.putText(result, ("Center: (" + samplecenter.x + ", " + samplecenter.y + ")"), new Point((samplecenter.x + 20), (samplecenter.y + 18)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
                Imgproc.putText(result, ("Angle: " + Math.round(sampleangle)), new Point((samplecenter.x + 20), (samplecenter.y + 36)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
                Imgproc.putText(result, ("Grab Angle: " + Math.round(grabangle)), new Point((samplecenter.x + 20), (samplecenter.y + 54)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
                Imgproc.putText(result, String.format("Distance: %.2f\n", getDistance()), new Point((samplecenter.x + 20), (samplecenter.y - 54)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
                Imgproc.putText(result, String.format("ServoExt: %.2f\n", getServoExt()), new Point((samplecenter.x + 20), (samplecenter.y - 36)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
                Imgproc.putText(result, String.format("StrafeDist: %.2f\n", getStrafeDist()), new Point((samplecenter.x + 20), (samplecenter.y - 18)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);
                Imgproc.putText(result, ("Bottom Y: " + Math.round(bottomy)), new Point((samplecenter.x + 20), (samplecenter.y - 72)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, green, 1);

                Imgproc.circle(result, testpoint, 2, white, 5);
                Imgproc.putText(result, ("Center: (" + Math.round(testpoint.x) + ", " + Math.round(testpoint.y) + ")"), new Point((testpoint.x - 40), (testpoint.y + 15)),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.3, white, 1);

                Equations();

                Log.d("Camera", "Center, Grab Angle: (" + samplecenter.x + ", " + samplecenter.y + "), " + grabangle);
            }

            if (scannow) {
                return result;
            } else {
                return input;
            }
        }

        public void Equations() {
            double reversey = 480 - samplecenter.y;

            bottomy = reversey;

            double Llim;
            double Rlim;

            double strafedist = 0;

            if (grabangle == 0) {
                Llim = 275; //270
                Rlim = 365; //390
            } else {
                Llim = 290; //290
                Rlim = 350; //350
            }

            double distance = ((0.0000000001282) * (Math.pow(reversey, 4))) - ((0.0000001066) * (Math.pow(reversey, 3))) +
                    ((0.00005792) * (Math.pow(reversey, 2))) + (0.0102 * reversey) + 2.75;//2.75 //3 //4.5 //0.5 //3.9049

            double servoext = ((-0.00000161) * (Math.pow(distance, 6))) + ((0.00009517) * (Math.pow(distance, 5))) -
                    ((0.002263) * (Math.pow(distance, 4))) + ((0.02786) * (Math.pow(distance, 3))) - ((0.1866) *
                    (Math.pow(distance, 2))) + (0.6626 * distance) - 0.8; //-.85///75 //0.8491

            double pixL = ((-0.0002801) * (Math.pow(distance, 7))) + ((0.0198) *
                    (Math.pow(distance, 6))) - ((0.5866) * (Math.pow(distance, 5))) +
                    ((9.4341) * (Math.pow(distance, 4))) - ((88.7643) * (Math.pow(distance, 3)))
                    + ((487.8807) * (Math.pow(distance, 2))) - (1450.6043 * distance) + 1852.021;
            double pixR = ((-0.0004808) * (Math.pow(distance, 5))) +
                    ((0.02754) * (Math.pow(distance, 4))) - ((0.6007) * (Math.pow(distance, 3)))
                    + ((6.281) * (Math.pow(distance, 2))) - (33.592 * distance) + 124.3077;

            if ((samplecenter.x < Llim) || (samplecenter.x > Rlim)) {
                if (samplecenter.x < Llim) {
                    strafedist = (samplecenter.x - 320) / Math.abs(pixL);
                } else {
                    strafedist = (samplecenter.x - 320) / Math.abs(pixR);
                }
            } else {
                strafedist = 0;
            }

            sampledist = distance;
            servoextend = servoext;
            disttostrafe = strafedist;
        }

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
    //endregion
}
