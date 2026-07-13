package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//@Disabled
@TeleOp(name = "Deep_TeleOp", group = "Opmode")
public class Deep_TeleOp extends OpMode {

    DeepRoboConstants robo = new DeepRoboConstants();
    Reset_Recal_Nav zero = new Reset_Recal_Nav();

    private ElapsedTime runtime = new ElapsedTime();

    //region - Variables
    double x;
    double y;
    double z;
    double l;

    boolean lifttohigh = false;
    boolean lifttoreset = false;
    boolean lifttowall = false;
    boolean lifttobar = false;
    boolean lifttolow = false;
    boolean lifttogap = false;
    boolean liftbaronce = false;
    boolean hang = false;
    boolean hangready = false;
    double hangtime = 0;
    double armtime = 0;
    boolean poke = false;
    boolean canpoke = true;
    boolean constanthang = false;
    boolean barscoring = false;
    boolean wallgrab = false;
    boolean postbar = false;
    boolean postdrop = false;
    double droptime = 0;
    boolean canautograb = true;

    boolean speed = true;
    boolean clawtoggleopen = false;
    boolean clawautoready = false;
    boolean intakegrab = false;
    boolean intakeout = false;

    boolean armback = false;
    boolean barreset = false;
    boolean barmove = false;

    double barmovetime = 0;

    boolean cangrab = true;
    boolean barscore = false;
    boolean bardrop = false;
    boolean specflip = false;
    boolean clawcheck = false;
    boolean flipready = true;

    double g1lsbtime = 0;
    double g1trigtime = 0;
    double g2trigtime = 0;
    double lifttime = 0;
    double clawautotime = 0;
    double afterintakein = 0;
    double autobartime = 0;

    boolean intakein = true;
    double grabtime = 0;

    boolean clawautocount = false;

    double wristtime = 0;
    double wristturn = 0;
    double fliptime = 0;
    double colortime = 0;

    double intakescalar = 1;
    double stall = 0;

    double walltime = 0;
    double wristpos = 0;

    boolean firstloop = false;

    SparkFunOTOS.Pose2D barrelease = new SparkFunOTOS.Pose2D(0, 0, 0);

    SparkFunOTOS.Pose2D BotPos = new SparkFunOTOS.Pose2D(0, 0, 0);
    SparkFunOTOS.Pose2D OTOSpos;
    //Pose2D ODOpos;
    //endregion

    @Override
    public void init() {
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

        robo.arml = hardwareMap.get(Servo.class, "arml");
        robo.armr = hardwareMap.get(Servo.class, "armr");

        robo.inExtL = hardwareMap.get(Servo.class, "intakeoutl");
        robo.inExtR = hardwareMap.get(Servo.class, "intakeoutr");

        robo.intakeclaw = hardwareMap.get(Servo.class, "intakeclaw");
        robo.claw = hardwareMap.get(Servo.class, "claw");
        robo.clawrotate = hardwareMap.get(Servo.class, "clawrotate");
        robo.intakepivot = hardwareMap.get(Servo.class, "intakepivot");
        robo.intakewrist = hardwareMap.get(Servo.class, "intakewrist");

        robo.otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        //robo.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        robo.Bus = hardwareMap.get(AnalogInput.class, "ultrab");
        robo.Rus = hardwareMap.get(AnalogInput.class, "ultrar");
        robo.Lus = hardwareMap.get(AnalogInput.class, "ultral");

        robo.liftMag = hardwareMap.get(DigitalChannel.class, "lift_mag");
        robo.clawbeam = hardwareMap.get(DigitalChannel.class, "clawbeam");

        robo.pin0 = hardwareMap.digitalChannel.get("color0");
        robo.pin1 = hardwareMap.digitalChannel.get("color1");

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
        //endregion

        //region - Initialize Robot
        robo.fl.setPower(0);
        robo.fr.setPower(0);
        robo.bl.setPower(0);
        robo.br.setPower(0);

        robo.lift1.setPower(0);
        robo.lift2.setPower(0);
        robo.lift3.setPower(0);

        robo.configureOtos();
        //robo.configureOdo();
        //robo.odo.resetPosAndIMU();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        runtime.reset();
        firstloop = true;
        //endregion
    }

    public void loop() {
        if (firstloop) {
            robo.arml.setPosition(robo.armltransfer);
            robo.armr.setPosition(robo.armrtransfer);
            robo.inExtL.setPosition(robo.intakeleftin);
            robo.inExtR.setPosition(robo.intakerightin);
            robo.intakepivot.setPosition(robo.intakepivotback);
            robo.claw.setPosition(robo.clawautograb);
            robo.intakeclaw.setPosition(robo.intakeopen);
            robo.clawrotate.setPosition(robo.clawrotatetransfer);
            robo.intakewrist.setPosition(robo.intakewrist0);
            firstloop = false;
            hang = false;
            hangready = true;
        }

        OTOSpos = robo.otos.getPosition();

        //region - Team Color
        if (robo.pin0.getState() && robo.pin1.getState()) {
            robo.colorseen = "Yellow";
        } else if (robo.pin0.getState()) {
            robo.colorseen = "Blue";
        } else if (robo.pin1.getState()) {
            robo.colorseen = "Red";
        } else {
            robo.colorseen = "None";
        }

        if (gamepad1.y && (runtime.milliseconds() > colortime)) {
            colortime = runtime.milliseconds() + 250;

            if (robo.teamcolor.matches("Red")) {
                robo.teamcolor = "Blue";
            } else if (robo.teamcolor.matches("Blue")) {
                robo.teamcolor = "Red";
            }
        }

        if (robo.inExtL.getPosition() == robo.intakeleftout) {
            if ((robo.teamcolor.matches("Red") && robo.colorseen.matches("Blue")) ||
                    (robo.teamcolor.matches("Blue") && robo.colorseen.matches("Red"))) {
                cangrab = false;
            } else {
                cangrab = true;
            }

            intakegrab = false;
        } else {
            cangrab = true;

            if ((robo.inExtL.getPosition() == robo.intakeleftin) && (runtime.milliseconds() > afterintakein)) {
                clawautoready = true;
                afterintakein = 0;
            } else {
                clawautoready = false;
            }
        }
        //endregion

        //region - Nav
        /*OTOSpos = robo.otos.getPosition();
        ODOpos = robo.odo.getPosition();

        BotPos.x = (OTOSpos.x + ODOpos.getX(DistanceUnit.INCH)) / 2;
        BotPos.y = (OTOSpos.y + ODOpos.getY(DistanceUnit.INCH)) / 2;
        BotPos.h = (OTOSpos.h + ODOpos.getHeading(AngleUnit.DEGREES)) / 2;

        if (gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y) {
            zero.resetnavigation(robo.otos, robo.odo, true, true, true, false);
        }*/
        //endregion

        //region - Set X, Y, & Z to joysticks
        x = gamepad1.right_stick_x;
        y = -gamepad1.left_stick_y;
        z = gamepad1.left_stick_x;

        if (robo.inExtL.getPosition() == robo.intakeleftin) {
            intakescalar = 1;
        } else if (robo.inExtL.getPosition() == robo.intakeleftout) {
            intakescalar = 0.75; // 0.65
        }

        robo.flpower = Range.clip(y + x + z, -1, 1);
        robo.frpower = Range.clip(y - x - z, -1, 1);
        robo.blpower = Range.clip(y + x - z, -1, 1);
        robo.brpower = Range.clip(y - x + z, -1, 1);

        //region - Slow/Fast
        if (gamepad1.left_stick_button && ((runtime.milliseconds() - g1lsbtime) > 500)) {
            g1lsbtime = runtime.milliseconds();

            if (speed) {
                robo.power = robo.fullpower;
                speed = false;
            } else {
                robo.power = robo.halfpower;
                speed = true;
            }
        }
        //endregion

        robo.fl.setPower(robo.flpower * robo.power * intakescalar);
        robo.fr.setPower(robo.frpower * robo.power * intakescalar);
        robo.bl.setPower(robo.blpower * robo.power);
        robo.br.setPower(robo.brpower * robo.power);
        //endregion

        //region - Intake Extend Out & Pivot
        if (gamepad2.a || gamepad1.dpad_left) {
            robo.inExtL.setPosition(robo.intakeleftin);
            robo.inExtR.setPosition(robo.intakerightin);
            robo.intakepivot.setPosition(robo.intakepivotback);

            if (!barscoring) {
                robo.arml.setPosition(robo.armltransfer);
                robo.armr.setPosition(robo.armrtransfer);
                robo.clawrotate.setPosition(robo.clawrotatetransfer);
                robo.claw.setPosition(robo.clawautograb);
            }

            robo.intakewrist.setPosition(robo.intakewrist0);
            robo.intakeclaw.setPosition(robo.intakeclosed);
            afterintakein = runtime.milliseconds() + 250;
            clawautocount = true;
            clawcheck = true;

            intakegrab = false;
            armback = false;
            specflip = false;

            intakein = true;
            grabtime = runtime.milliseconds();

            if (postbar) {
                lifttogap = false;
                lifttime = runtime.milliseconds();
                lifttoreset = true;
                postbar = false;
            }
        } else if (gamepad2.y || gamepad1.dpad_right) {
            robo.inExtL.setPosition(robo.intakeleftout);
            robo.inExtR.setPosition(robo.intakerightout);
            robo.intakepivot.setPosition(robo.intakepivotdown);
            robo.intakewrist.setPosition(robo.intakewrist0);
            robo.intakeclaw.setPosition(robo.intakeclosed);

            if (!barscoring) {
                robo.claw.setPosition(robo.clawopenfull);
            }

            intakein = false;
            clawtoggleopen = false;
            clawautoready = false;
            intakeout = true;
            wristturn = 0;
        } else if ((gamepad1.left_trigger > 0.75) && (gamepad1.right_trigger < 0.1)) {
            robo.intakepivot.setPosition(robo.intakepivotup);
        } else if ((gamepad1.right_trigger > 0.75) && (gamepad1.left_trigger < 0.1)) {
            robo.intakepivot.setPosition(robo.intakepivotdown);
            clawtoggleopen = false;
            wristturn = 0;
        }

        if ((!(gamepad2.y || gamepad1.dpad_right)) && intakeout) {
            robo.intakeclaw.setPosition(robo.intakeopen);
            intakeout = false;
        }

        if ((gamepad1.x || gamepad1.b) && ((runtime.milliseconds() - wristtime) > 150)) { //100
            wristtime = runtime.milliseconds();

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

            robo.intakewrist.setPosition(wristpos);
        }

        //region - Manual Ext
        if (gamepad1.right_stick_y < -0.75) {
            robo.inExtL.setPosition(robo.inExtL.getPosition() - 0.01);
            robo.inExtR.setPosition(robo.inExtR.getPosition() + 0.01);
        } else if (gamepad1.right_stick_y > 0.75) {
            robo.inExtL.setPosition(robo.inExtL.getPosition() + 0.01);
            robo.inExtR.setPosition(robo.inExtR.getPosition() - 0.01);
        }

        if (robo.inExtL.getPosition() > robo.intakeleftin) {
            robo.inExtL.setPosition(robo.intakeleftin);
        } else if (robo.inExtL.getPosition() < robo.intakeleftout) {
            robo.inExtL.setPosition(robo.intakeleftout);
        }

        if (robo.inExtR.getPosition() < robo.intakerightin) {
            robo.inExtR.setPosition(robo.intakerightin);
        } else if (robo.inExtR.getPosition() > robo.intakerightout) {
            robo.inExtR.setPosition(robo.intakerightout);
        }
        //endregion

        if (robo.lift1.getCurrentPosition() < robo.lifthanglim) {
            if (gamepad2.left_bumper && (!gamepad2.right_bumper)) {
                robo.intakepivot.setPosition(robo.intakepivotback);
            } else if (gamepad2.right_bumper && (!gamepad2.left_bumper)) {
                robo.intakepivot.setPosition(robo.intakepivotup);
            }
        }
        //endregion

        //region - Claw
        if (gamepad2.left_trigger > 0.5) {
            g2trigtime = runtime.milliseconds();
            robo.claw.setPosition(robo.clawclosed);

            if (robo.lift1.getCurrentPosition() < robo.Lgrab) {
                while ((runtime.milliseconds() - g2trigtime) < 200) {
                    stall = stall + 1;
                }
            }

            robo.intakeclaw.setPosition(robo.intakeopen);
        } else if ((gamepad2.right_trigger > 0.5) && cangrab) {
            g2trigtime = runtime.milliseconds();
            robo.intakeclaw.setPosition(robo.intakeclosed);

            if (robo.lift1.getCurrentPosition() < robo.Lgrab) {
                while ((runtime.milliseconds() - g2trigtime) < 200) {
                    stall = stall + 1;
                }
            }

            robo.claw.setPosition(robo.clawopenfull);
        }

        if ((gamepad1.a) && /*cangrab &&*/ ((runtime.milliseconds() - g1trigtime) > 300)) {
            g1trigtime = runtime.milliseconds();

            if (!clawtoggleopen) {
                robo.intakeclaw.setPosition(robo.intakeclosed);

                while ((runtime.milliseconds() - g1trigtime) < 300) { //200
                    stall = stall + 1;
                }

                robo.claw.setPosition(robo.clawautograb);

                while ((runtime.milliseconds() - g1trigtime) < 100) { //200
                    stall = stall + 1;
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

                if (!(robo.colorseen.matches("Yellow") || robo.colorseen.matches(robo.teamcolor))) {
                    robo.intakeclaw.setPosition(robo.intakeopen);
                    robo.intakepivot.setPosition(robo.intakepivotdown);
                    clawtoggleopen = false;
                } else {
                    robo.inExtL.setPosition(robo.intakeleftin);
                    robo.inExtR.setPosition(robo.intakerightin);
                    robo.intakepivot.setPosition(robo.intakepivotback);
                    robo.arml.setPosition(robo.armltransfer);
                    robo.armr.setPosition(robo.armrtransfer);
                    robo.clawrotate.setPosition(robo.clawrotatetransfer);
                    robo.claw.setPosition(robo.clawautograb);
                    robo.intakewrist.setPosition(robo.intakewrist0);
                    robo.intakeclaw.setPosition(robo.intakeclosed);
                    afterintakein = runtime.milliseconds() + 250;

                    //if (robo.colorseen.matches(robo.teamcolor)) {
                        clawautocount = true;
                        clawcheck = true;
                    //}

                    intakegrab = false;
                    armback = false;
                    specflip = false;

                    clawtoggleopen = true;

                    if (postbar) {
                        lifttogap = false;
                        lifttime = runtime.milliseconds();
                        lifttoreset = true;
                        postbar = false;
                    }
                }

                stall = 0;
            } else {
                robo.claw.setPosition(robo.clawclosed);

                while ((runtime.milliseconds() - g1trigtime) < 200) {
                    stall = stall + 1;
                }

                robo.intakeclaw.setPosition(robo.intakeopen);
                clawtoggleopen = false;
                stall = 0;
            }
        }

        if (armback) {
            if (!robo.clawbeam.getState()) {
                clawautoready = true;
                clawautocount = true;
            } else {
                clawautoready = false;
                clawautocount = false;
            }
        }

        if (!robo.clawbeam.getState() && clawautoready && clawautocount && clawcheck &&
                (((runtime.milliseconds() - walltime) > 500) || (intakein && ((runtime.milliseconds() - grabtime) > 200))) && canautograb) { //2000
            clawautotime = runtime.milliseconds();
            robo.claw.setPosition(robo.clawclosed);
            intakegrab = true;
            clawcheck = false;

            if (armback && ((runtime.milliseconds() - walltime) > 1000)) {
                specflip = true;
            } else {
                specflip = false;
            }
        }

        if (((runtime.milliseconds() - clawautotime) > 200) && intakegrab) {
            robo.intakeclaw.setPosition(robo.intakeopen);
            clawtoggleopen = false;
            clawautoready = false;
            clawautocount = false;

            if (specflip) {
                if (liftbaronce) {
                    lifttobar = true;
                    liftbaronce = false;
                    fliptime = runtime.milliseconds();
                }

                if ((runtime.milliseconds() - fliptime) > 100) {
                    robo.arml.setPosition(robo.armlbarpt1);
                    robo.armr.setPosition(robo.armrbarpt1);

                    if ((runtime.milliseconds() - fliptime) > 400) {
                        robo.clawrotate.setPosition(robo.clawrotatebar2);
                        specflip = false;
                        clawautoready = true;
                        clawautocount = true;
                        intakegrab = false;
                    }
                }
            }
        }
        //endregion

        //region - Arm
        if (gamepad2.right_stick_y > 0.5) {
            robo.arml.setPosition(robo.armltransfer);
            robo.armr.setPosition(robo.armrtransfer);
            robo.claw.setPosition(robo.clawautograb);
            hangready = true;
            robo.clawrotate.setPosition(robo.clawrotatetransfer);
        } else if (gamepad2.right_stick_y < -0.5) {
            robo.arml.setPosition(robo.armldeliver);
            robo.armr.setPosition(robo.armrdeliver);
            robo.clawrotate.setPosition(robo.clawrotatedeliverb);
            clawautoready = false;
            //hangready = false;
        } else if ((gamepad2.right_stick_button || (gamepad1.right_bumper && (!gamepad1.left_bumper))) && flipready &&
                (robo.lift1.getCurrentPosition() < robo.lifthanglim)) {
            if (!armback) {
                robo.arml.setPosition(robo.armlback1);
                robo.armr.setPosition(robo.armrback1);
            }

            robo.clawrotate.setPosition(robo.clawrotatebackgrab);

            clawautoready = false;
            clawautocount = false;
            clawcheck = false;
            specflip = false;
            intakegrab = false;
            lifttowall = true;
            armback = true;
            liftbaronce = true;
            hangready = false;
            barscoring = true;
            wallgrab = true;
            canautograb = false;
        } else if (gamepad2.left_stick_button || gamepad1.right_stick_button) {
            if (runtime.milliseconds() > barmovetime){
                barmovetime = runtime.milliseconds() + 200;

                if (barmove) {
                    barmove = false;
                } else {
                    barmove = true;
                }
            }

            if (barmove) {
                robo.arml.setPosition(robo.armlmove);
                robo.armr.setPosition(robo.armrmove);
            } else {
                robo.arml.setPosition(robo.armltransfer);
                robo.armr.setPosition(robo.armrtransfer);
            }

            hangready = false;
        } else if ((gamepad2.x || (gamepad1.left_bumper && (!gamepad1.right_bumper))) && barscore &&
                (robo.lift1.getCurrentPosition() < robo.lifthanglim)) {
            robo.arml.setPosition(robo.armlbarpt2);
            robo.armr.setPosition(robo.armrbarpt2);
            autobartime = runtime.milliseconds() + 600; //400
            bardrop = true;
            barscore = false;
            armback = false;
            clawautoready = false;
            clawautocount = false;
            flipready = false;

            robo.otos.resetTracking();
            robo.otos.calibrateImu();
            /*DriveStop();

            robo.arml.setPosition(robo.armlbarpt2);
            robo.armr.setPosition(robo.armlbarpt2);
            barscore = false;

            zero.resetnavigation(robo.otos, robo.odo, true, true, true, true);
            barrelease = robo.otos.getPosition();

            bardrop = true;

            robo.power = robo.halfpower;
        }

        if ((robo.otos.getPosition().y < robo.bartolerance) && bardrop) {
            robo.claw.setPosition(robo.clawopenfull);
            bardrop = false;
            robo.power = robo.fullpower;*/

            hangready = false;
        }

        if (bardrop && (runtime.milliseconds() > autobartime)) {
            robo.claw.setPosition(robo.clawopenfull);
            barrelease = robo.otos.getPosition();
            bardrop = false;
            barreset = true;
        }

        if (wallgrab && !gamepad2.right_stick_button) {
            droptime = runtime.milliseconds();
            robo.claw.setPosition(robo.clawautograb);
            wallgrab = false;
            postdrop = true;
        }

        if (((runtime.milliseconds() - droptime) > 400) && postdrop) {
            clawautoready = true;
            clawautocount = true;
            clawcheck = true;
            postdrop = false;
            canautograb = true;
        }

        if ((runtime.milliseconds() > (autobartime + 200)) && (robo.otos.getPosition().y < robo.bartolerance) && barreset) {
            robo.arml.setPosition(robo.armltransfer);
            robo.armr.setPosition(robo.armrtransfer);
            robo.clawrotate.setPosition(robo.clawrotatetransfer);
            robo.claw.setPosition(robo.clawautograb);
            barreset = false;
            flipready = true;
            hangready = true;
            //lifttogap = true;
            postbar = true;
            barscoring = false;
        }
        //endregion

        //region - All Lift Stuff
        if (gamepad2.b){
            hangready = true;
            robo.clawrotate.setPosition(robo.clawrotatehang);
            robo.claw.setPosition(robo.clawautograb);
        }

        if (hangready && gamepad2.b  && gamepad2.right_bumper && gamepad2.left_bumper &&// had gamepad2.bumpers
                ((runtime.milliseconds() - hangtime) > 500) && (robo.lift1.getCurrentPosition() >= robo.lifthanglim)) {
            hangtime = runtime.milliseconds();

            if (!hang) {
                armtime = runtime.milliseconds();
                //add other hang trigger stuff here
                l = Range.clip((-gamepad2.left_stick_y), -0.5, 0.5); //0.4

                robo.clawrotate.setPosition(robo.clawrotatehang);
                robo.claw.setPosition(robo.clawautograb);
                robo.arml.setPosition(robo.armlhang);
                robo.armr.setPosition(robo.armrhang);

                hang = true;

                if (canpoke) {
                    poke = true;
                    canpoke = false;
                }
            } else {
                l = -gamepad2.left_stick_y;

                robo.clawrotate.setPosition(robo.clawrotatetransfer);
                robo.claw.setPosition(robo.clawautograb);
                robo.arml.setPosition(robo.armltransfer);
                robo.armr.setPosition(robo.armrtransfer);

                hang = false;
            }
        } else if (!hang) {
            l = -gamepad2.left_stick_y;
        }

        if (((runtime.milliseconds() - armtime) > 500) && poke) {
            robo.arml.setPosition(robo.armltransfer);
            robo.armr.setPosition(robo.armrtransfer);
            robo.clawrotate.setPosition(robo.clawrotatetransfer);

            poke = false;
        }

        if ((gamepad2.left_stick_y > 0.5) && (robo.arml.getPosition() < robo.armltransfer)) {
            robo.arml.setPosition(robo.armltransfer);
            robo.armr.setPosition(robo.armrtransfer);
        }

        //region - Mag Reset Lift
        if ((!robo.liftMag.getState()) && (!hang)) {
            if ((Math.abs(robo.lift1.getCurrentPosition()) > 0) && (Math.abs(gamepad2.left_stick_y) < 0.05)) {
                LiftReset();
            }
            robo.liftreset = true;
        } else {
            robo.liftreset = false;
        }
        //endregion

        //region - Lift Power (0 if down, max, or up to start)
        if (((robo.liftreset && (l < 0)) || ((robo.lift1.getCurrentPosition() > robo.liftmax) && (l > 0)) ||
                ((robo.lift1.getCurrentPosition() == 0) && robo.liftMag.getState() && (l > 0))) && (!hang)) {
            robo.lift1.setPower(0);
            robo.lift2.setPower(0);
            robo.lift3.setPower(0);
        } else if (!lifttohigh && !lifttoreset && !lifttowall && !hang) {
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                l = -gamepad2.left_stick_y;
            } else if (robo.lift1.getCurrentPosition() > robo.liftholdmin) {
                l = robo.liftholdpower;
            }

            robo.lift1.setPower(l);
            robo.lift2.setPower(l);
            robo.lift3.setPower(l);
        } else if (hang) {
            if (constanthang) {
                robo.lift1.setPower(-0.6); //0.4
                robo.lift2.setPower(-0.6);
                robo.lift3.setPower(-0.6);
            } else if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                l = -gamepad2.left_stick_y;
            } else {
                l = robo.liftholdpower;
            }

            robo.lift1.setPower(l);
            robo.lift2.setPower(l);
            robo.lift3.setPower(l);
        }
        //endregion

        //region - Lift Buttons
        if ((Math.abs(gamepad2.left_stick_y) < 0.05) && (!hang)) {
            if (gamepad2.dpad_up || gamepad1.dpad_up) {
                robo.arml.setPosition(robo.armldeliver);
                robo.armr.setPosition(robo.armrdeliver);
                robo.clawrotate.setPosition(robo.clawrotatedeliverb);
                robo.claw.setPosition(robo.clawclosed);
                lifttohigh = true;
                barreset = false;
            } else if ((gamepad2.dpad_down || gamepad1.dpad_down) && robo.liftMag.getState()) {
                robo.arml.setPosition(robo.armltransfer);
                robo.armr.setPosition(robo.armrtransfer);
                robo.clawrotate.setPosition(robo.clawrotatetransfer);
                robo.claw.setPosition(robo.clawautograb);
                lifttime = runtime.milliseconds();
                lifttoreset = true;
            } else if (gamepad2.dpad_right) {
                robo.arml.setPosition(robo.armldeliver);
                robo.armr.setPosition(robo.armrdeliver);
                robo.clawrotate.setPosition(robo.clawrotatedeliverb);
                robo.claw.setPosition(robo.clawclosed);
                lifttolow = true;
                barreset = false;
            } else if (gamepad2.dpad_left) {
                robo.arml.setPosition(robo.armltransfer);
                robo.armr.setPosition(robo.armrtransfer);
                robo.clawrotate.setPosition(robo.clawrotatetransfer);
                robo.claw.setPosition(robo.clawautograb);
                lifttogap = true;
                barreset = false;
            }

            if (!lifttoreset) {
                if (lifttohigh) {
                    if (robo.lift1.getCurrentPosition() < robo.lifthigh) {
                        robo.lift1.setPower(robo.fullpower);
                        robo.lift2.setPower(robo.fullpower);
                        robo.lift3.setPower(robo.fullpower);
                    } else if (robo.lift1.getCurrentPosition() >= robo.lifthigh) {
                        robo.lift1.setPower(robo.liftholdpower);
                        robo.lift2.setPower(robo.liftholdpower);
                        robo.lift3.setPower(robo.liftholdpower);
                        lifttohigh = false;
                    }
                } else if (lifttowall) {
                    if (robo.lift1.getCurrentPosition() < robo.Lgrab) {
                        robo.lift1.setPower(robo.liftlowpower);
                        robo.lift2.setPower(robo.liftlowpower);
                        robo.lift3.setPower(robo.liftlowpower);
                    } else if (robo.lift1.getCurrentPosition() >= robo.Lgrab) {
                        robo.lift1.setPower(robo.liftholdpower);
                        robo.lift2.setPower(robo.liftholdpower);
                        robo.lift3.setPower(robo.liftholdpower);
                        lifttowall = false;

                        robo.arml.setPosition(robo.armlbackgrab);
                        robo.armr.setPosition(robo.armrbackgrab);
                    }
                } else if (lifttobar) {
                    if (robo.lift1.getCurrentPosition() < robo.Lbar2) {
                        robo.lift1.setPower(robo.liftbarpower);
                        robo.lift2.setPower(robo.liftbarpower);
                        robo.lift3.setPower(robo.liftbarpower);

                        robo.claw.setPosition(robo.clawclosed);
                        barscore = true;
                    } else if (robo.lift1.getCurrentPosition() >= robo.Lbar2) {
                        robo.lift1.setPower(robo.liftholdpower);
                        robo.lift2.setPower(robo.liftholdpower);
                        robo.lift3.setPower(robo.liftholdpower);
                        lifttobar = false;
                        lifttoreset = true;
                        lifttime = runtime.milliseconds();
                    }
                } else if (lifttolow) {
                    if (robo.lift1.getCurrentPosition() < robo.liftlow) {
                        robo.lift1.setPower(robo.fullpower);
                        robo.lift2.setPower(robo.fullpower);
                        robo.lift3.setPower(robo.fullpower);
                    } else if (robo.lift1.getCurrentPosition() >= robo.liftlow) {
                        robo.lift1.setPower(robo.liftholdpower);
                        robo.lift2.setPower(robo.liftholdpower);
                        robo.lift3.setPower(robo.liftholdpower);
                        lifttolow = false;
                    }
                } else if (lifttogap) {
                    if (robo.lift1.getCurrentPosition() < robo.liftgap) {
                        robo.lift1.setPower(robo.fullpower);
                        robo.lift2.setPower(robo.fullpower);
                        robo.lift3.setPower(robo.fullpower);
                    } else if (robo.lift1.getCurrentPosition() >= robo.liftgap) {
                        robo.lift1.setPower(robo.liftholdpower);
                        robo.lift2.setPower(robo.liftholdpower);
                        robo.lift3.setPower(robo.liftholdpower);
                        lifttogap = false;
                    }
                }
            } else if ((runtime.milliseconds() - lifttime) < robo.liftdowntimelimit) {
                if (robo.lift1.getCurrentPosition() > robo.liftdown) {
                    robo.lift1.setPower(-robo.fullpower);
                    robo.lift2.setPower(-robo.fullpower);
                    robo.lift3.setPower(-robo.fullpower);
                } else if (!robo.liftMag.getState()) {
                    if (!specflip) {
                        robo.arml.setPosition(robo.armltransfer);
                        robo.armr.setPosition(robo.armrtransfer);
                    }

                    lifttoreset = false;
                    LiftStop();
                }
            }
        } else if (hang) {
            //if (Math.abs(gamepad2.left_stick_y) > 0.5) {
                if (robo.lift1.getCurrentPosition() < robo.lifthanglim) {
                    constanthang = true;
                } else {
                    l = -gamepad2.left_stick_y;
                    constanthang = false;
                }
        } else if (Math.abs(gamepad2.left_stick_y) > 0.1) {
            lifttohigh = false;
            lifttoreset = false;
            lifttobar = false;
            lifttowall = false;
            lifttolow = false;
            lifttogap = false;
        }
        //endregion
        //endregion

        //region - Telemetry
        telemetry.addData("Hang: ", hang);
        telemetry.addData("Team Color: ", robo.teamcolor);
        telemetry.addData("Color Seen: ", robo.colorseen);
        telemetry.addData("Lift Pos: ", robo.lift1.getCurrentPosition());
        telemetry.addData("Lift Mag: ", robo.liftMag.getState());
        telemetry.addData("Beam State: ", robo.clawbeam.getState());
        telemetry.addData("Otos X, Y, Z: ", OTOSpos.x + ", " + OTOSpos.y + ", " + OTOSpos.h);
        telemetry.addData("Claw Rotate: ", robo.clawrotate.getPosition());
        telemetry.update();
        //endregion
    }

    public void stop() {
        DriveStop();
        LiftStop();
    }

    public void LiftReset() {
        robo.lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void LiftStop() {
        robo.lift1.setPower(0);
        robo.lift2.setPower(0);
        robo.lift3.setPower(0);
    }

    public void DriveStop() {
        robo.fl.setPower(0);
        robo.fr.setPower(0);
        robo.bl.setPower(0);
        robo.br.setPower(0);
    }
}
