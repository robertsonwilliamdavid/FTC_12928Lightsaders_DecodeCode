package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@Disabled
@Autonomous(name = "Deep_Otos_Test", group = "LinearOpmode")
public class Deep_Otos_Test extends LinearOpMode {

    DeepRoboConstants robo = new DeepRoboConstants();

    private ElapsedTime runtime = new ElapsedTime();

    boolean forward = true;
    double drivetimeout = 0;

    @Override
    public void runOpMode() {
        //region - Hardware Map
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robo.fl = hardwareMap.get(DcMotor.class, "fl");
        robo.fr = hardwareMap.get(DcMotor.class, "fr");
        robo.bl = hardwareMap.get(DcMotor.class, "bl");
        robo.br = hardwareMap.get(DcMotor.class, "br");

        robo.lift1 = hardwareMap.get(DcMotor.class, "lift1");
        robo.lift2 = hardwareMap.get(DcMotor.class, "lift2");
        robo.lift3 = hardwareMap.get(DcMotor.class, "lift3");

        robo.otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        robo.liftMag = hardwareMap.get(DigitalChannel.class, "lift_mag");

        robo.arml = hardwareMap.get(Servo.class, "arml");
        robo.armr = hardwareMap.get(Servo.class, "armr");

        robo.inExtL = hardwareMap.get(Servo.class, "intakeoutl");
        robo.inExtR = hardwareMap.get(Servo.class, "intakeoutr");

        robo.intakeclaw = hardwareMap.get(Servo.class, "intakeclaw");
        robo.claw = hardwareMap.get(Servo.class, "claw");
        robo.clawrotate = hardwareMap.get(Servo.class, "clawrotate");
        robo.intakepivot = hardwareMap.get(Servo.class, "intakepivot");
        robo.intakewrist = hardwareMap.get(Servo.class, "intakewrist");

        robo.fl.setDirection(DcMotor.Direction.REVERSE);
        robo.bl.setDirection(DcMotor.Direction.REVERSE);

        robo.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        robo.claw.setPosition(robo.clawopenfull);
        robo.intakeclaw.setPosition(robo.intakeopen);
        robo.intakewrist.setPosition(robo.intakewrist0);
        robo.intakepivot.setPosition(robo.intakepivotback);
        robo.clawrotate.setPosition(robo.clawrotatetransfer);
        robo.arml.setPosition(robo.armltransfer);
        robo.armr.setPosition(robo.armrtransfer);

        robo.configureOtos();

        robo.otos.resetTracking();
        robo.otos.calibrateImu();

        robo.roboorigin = new SparkFunOTOS.Pose2D(0, 0, 0);

        robo.otos.setPosition(robo.roboorigin);

        telemetry.addData("Otos x: ", robo.otos.getPosition().x);
        telemetry.addData("Otos y: ", robo.otos.getPosition().y);
        telemetry.addData("Otos z: ", robo.otos.getPosition().h);
        telemetry.addData("Status", "Ready");
        telemetry.update();
        //endregion

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            robo.otosloc = robo.otos.getPosition();

            robo.location.x = robo.otosloc.x;
            robo.location.y = robo.otosloc.y;
            robo.location.h = robo.otosloc.h;

            /*
            if (forward) {
                if (robo.location.y < 40) {
                    robo.fl.setPower(0.5);
                    robo.fr.setPower(0.5);
                    robo.bl.setPower(0.5);
                    robo.br.setPower(0.5);

                    robo.inExtL.setPosition(robo.intakeleftout);
                    robo.inExtR.setPosition(robo.intakerightout);
                    robo.intakepivot.setPosition(robo.intakepivotdown);
                    robo.intakewrist.setPosition(robo.intakewrist45);
                    robo.intakeclaw.setPosition(robo.intakeclosed);

                    robo.arml.setPosition(robo.armldeliver);
                    robo.armr.setPosition(robo.armrdeliver);
                    robo.clawrotate.setPosition(robo.clawrotatedeliverb);
                    robo.claw.setPosition(robo.clawclosed);

                    if (robo.lift1.getCurrentPosition() < robo.lifthanglim) {
                        robo.lift1.setPower(1);
                        robo.lift2.setPower(1);
                        robo.lift3.setPower(1);
                    } else {
                        robo.lift1.setPower(robo.liftholdpower);
                        robo.lift2.setPower(robo.liftholdpower);
                        robo.lift3.setPower(robo.liftholdpower);
                    }
                } else {
                    forward = false;
                }
            } else {
                if (robo.location.y > 0) {
                    robo.fl.setPower(-0.5);
                    robo.fr.setPower(-0.5);
                    robo.bl.setPower(-0.5);
                    robo.br.setPower(-0.5);

                    robo.inExtL.setPosition(robo.intakeleftin);
                    robo.inExtR.setPosition(robo.intakerightin);
                    robo.intakepivot.setPosition(robo.intakepivotback);
                    robo.intakewrist.setPosition(robo.intakewrist0);
                    robo.intakeclaw.setPosition(robo.intakeopen);

                    robo.arml.setPosition(robo.armltransfer);
                    robo.armr.setPosition(robo.armrtransfer);
                    robo.clawrotate.setPosition(robo.clawrotatetransfer);
                    robo.claw.setPosition(robo.clawopenfull);

                    if (robo.liftMag.getState()) {
                        robo.lift1.setPower(-1);
                        robo.lift2.setPower(-1);
                        robo.lift3.setPower(-1);
                    } else {
                        robo.lift1.setPower(0);
                        robo.lift2.setPower(0);
                        robo.lift3.setPower(0);
                    }
                } else {
                    forward = true;
                }
            }
            */

            if (forward) {
                ServoArm(robo.armldeliver, robo.armrdeliver);
                ClawRot(robo.clawrotatedeliverb);
                Claw(robo.clawclosed);
                IntakeWrist(robo.intakewrist0);
                IntakePivot(robo.intakepivotscan);
                IntakeExt(robo.intakeleftin, robo.intakerightin);
                Intake(robo.intakeopen);

                drivetimeout = runtime.milliseconds() + 1300;
                while (/*(robo.lift1.getCurrentPosition() < robo.lifthigh) ||*/ (runtime.milliseconds() < drivetimeout)) {
                    if (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && (((Math.abs(robo.del3.x -
                            robo.location.x) > 0.25)) || (Math.abs(robo.del3.y - robo.location.y) > 0.25) ||
                            (Math.abs(robo.del3.h - robo.location.h) > 0.25))) {
                        XYZ(robo.del3, 10, true, false, 0.2); //0.8 //1, 0.5
                    }

                    /*if (runtime.milliseconds() > (drivetimeout - 1100)) {
                        LiftMulti(robo.lifthigh, (drivetimeout - 1200)); //-1300
                    }*/
                }

                DriveStop();
                //LiftStop(robo.lifthigh);
                Pos(robo.del3);

                forward = false;
            } else {
                sleep(1000);

                ServoArm(robo.armltransfer, robo.armrtransfer);
                ClawRot(robo.clawrotatescan);
                Claw(robo.clawopenfull);
                IntakeWrist(robo.intakewrist0);
                IntakePivot(robo.intakepivotscan);
                IntakeExt(robo.intakeleftin, robo.intakerightin);
                Intake(robo.intakeopen);

                sleep(100);

                drivetimeout = runtime.milliseconds() + 1300;
                while (/*(robo.lift1.getCurrentPosition() > robo.liftdown) ||*/ (runtime.milliseconds() < drivetimeout)) {
                    if (opModeIsActive() && (runtime.milliseconds() < drivetimeout) && (((Math.abs(robo.tosub1.x -
                            robo.location.x) > 0.25)) || (Math.abs(robo.tosub1.y - robo.location.y) > 0.25) ||
                            (Math.abs(robo.tosub1.h - robo.location.h) > 0.25))) {
                        XYZ(robo.tosub1, 10, true, false, 0.8); //1, 0.5
                    }

                    //LiftdownMulti(drivetimeout - 1300); //-1300
                }

                DriveStop();
                //LiftStop(robo.liftdown);
                Pos(robo.tosub1);

                forward = true;
            }

            Log.d("Position", "X, Y, Z: " + robo.location.x + ", " + robo.location.y + ", " + robo.location.h);
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

    public void LiftMulti(double liftupto, double uptime) {
        if ((robo.lift1.getCurrentPosition() < liftupto) && (runtime.milliseconds() < (uptime + 2000))) {
            robo.lift1.setPower(0.5);
            robo.lift2.setPower(0.5);
            robo.lift3.setPower(0.5);

            Log.d("LIFT", "Lift Pos: " + robo.lift1.getCurrentPosition());
        } else {
            LiftStop(liftupto);
        }
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
}
