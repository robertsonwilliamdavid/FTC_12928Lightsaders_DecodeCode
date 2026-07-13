package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
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
@TeleOp(name = "Deep_Axon_Test", group = "Opmode")
public class Deep_Axon_Test extends OpMode {

    DeepRoboConstants robo = new DeepRoboConstants();

    private ElapsedTime runtime = new ElapsedTime();

    double intakewristturn = 0;
    boolean clawtoggleopen = false;

    double g1rsbtime = 0;
    double g1trigtime = 0;
    double clawautotime = 0;

    double l;
    double stall = 0;




    @Override
    public void init() {
        //region - Hardware Map
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robo.testservo1 = hardwareMap.get(CRServo.class, "testservo1");
        robo.testservo2 = hardwareMap.get(CRServo.class, "testservo2");

        robo.testservo1encoder = hardwareMap.get(AnalogInput.class, "testservo1encoder");
        robo.testservo2encoder = hardwareMap.get(AnalogInput.class, "testservo2encoder");

        robo.arml = hardwareMap.get(Servo.class, "arml");
        robo.armr = hardwareMap.get(Servo.class, "armr");
        robo.intakeclaw = hardwareMap.get(Servo.class, "intakeclaw");
        robo.clawrotate = hardwareMap.get(Servo.class, "clawrotate");
        robo.inExtL = hardwareMap.get(Servo.class, "intakeoutl");
        robo.inExtR = hardwareMap.get(Servo.class, "intakeoutr");
        robo.intakepivot = hardwareMap.get(Servo.class, "intakepivot");
        robo.claw = hardwareMap.get(Servo.class, "claw");
        robo.lift1 = hardwareMap.get(DcMotor.class, "lift1");
        robo.lift2 = hardwareMap.get(DcMotor.class, "lift2");
        robo.lift3 = hardwareMap.get(DcMotor.class, "lift3");
        robo.intakewrist = hardwareMap.get(Servo.class, "intakewrist");

        robo.clawbeam = hardwareMap.get(DigitalChannel.class, "clawbeam");
        robo.liftMag = hardwareMap.get(DigitalChannel.class, "lift_mag");

        robo.lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        robo.lift3.setDirection(DcMotorSimple.Direction.REVERSE);

        robo.lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.lift3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robo.lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robo.intakewrist.setPosition(robo.intakewrist0);
        robo.arml.setPosition(robo.armlup);
        robo.armr.setPosition(robo.armrup);
        robo.inExtR.setPosition(robo.intakerightin);
        robo.inExtL.setPosition(robo.intakeleftin);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        runtime.reset();
        //endregion
    }

    public void loop() {
        //use this program for servos and lift, use TeleOpTest for driving and OTOS

        //find test servo position
        double testservo1pos = robo.testservo1encoder.getVoltage()/ 3.3 * 360;
        double testservo2pos = robo.testservo2encoder.getVoltage()/ 3.3 * 360;

        double holdPowerTestServo1 = -0.06;
        double holdPowerTestServo2 = -0.07;

        double PowerTestServo1 = holdPowerTestServo1;
        double PowerTestServo2 = holdPowerTestServo2;




        //region - Outtake
        //region - Lift
        l = -gamepad2.left_stick_y;
        robo.lift1.setPower(l);
        robo.lift2.setPower(l);
        robo.lift3.setPower(l);
        //endregion
//region test servos
        if (gamepad2.right_bumper){
            robo.testservo1.setPower(1);
            robo.testservo2.setPower(1);
        }
        if (gamepad2.left_bumper){
            robo.testservo1.setPower(-1);
            robo.testservo2.setPower(-1);
        }
        if (gamepad2.a){
            robo.testservo1.setPower(holdPowerTestServo1);
            robo.testservo2.setPower(holdPowerTestServo2);
        }
        if (gamepad2.y){

            testservo1pos = robo.testservo1encoder.getVoltage()/ 3.3 * 360;

            if (testservo1pos < 170){
                PowerTestServo1 = -.5;
            } else if (testservo1pos > 190) {
                PowerTestServo1 = .3;
            } else if ((170 < testservo1pos) && (testservo1pos < 190)){
                PowerTestServo1 = holdPowerTestServo1;
            }
            robo.testservo1.setPower(PowerTestServo1);
            telemetry.addData("Test Servo 1 Power: ", PowerTestServo1);

            telemetry.addData("Test Servo 1 Pos: ", testservo1pos);

            testservo2pos = robo.testservo2encoder.getVoltage()/ 3.3 * 360;

            if (testservo2pos < 170){
                PowerTestServo2 = -.5;
            } else if (testservo2pos > 200) {
                PowerTestServo2 = .3;
            } else if ((170 < testservo2pos) && (testservo2pos < 200)){
                PowerTestServo2 = holdPowerTestServo2;
            }
            robo.testservo2.setPower(PowerTestServo2);

            telemetry.addData("Test Servo 1 Power: ", PowerTestServo2);

            telemetry.addData("Test Servo 2 Pos: ", testservo2pos);


//            while (testservo2pos < 170){
//                testservo2pos = robo.testservo2encoder.getVoltage()/ 3.3 * 360;
//                robo.testservo2.setPower(-.1);
//
//            }
//            while (testservo2pos > 190) {
//                testservo2pos = robo.testservo2encoder.getVoltage()/ 3.3 * 360;
//                robo.testservo2.setPower(.1);
//            }

        }



        //region - Arm

        if (gamepad1.dpad_up) {
            robo.arml.setPosition(robo.arml.getPosition() + 0.002);
            robo.armr.setPosition(robo.armr.getPosition() - 0.002);
        } else if (gamepad1.dpad_down) {
            robo.arml.setPosition(robo.arml.getPosition() - 0.002);
            robo.armr.setPosition(robo.armr.getPosition() + 0.002);
        }
        //endregion

        //region - Claw Rotate
        if (gamepad1.y) {
            robo.clawrotate.setPosition(robo.clawrotate.getPosition() + 0.002);
        } else if (gamepad1.a) {
            robo.clawrotate.setPosition(robo.clawrotate.getPosition() - 0.002);
        }
        //endregion

        //region - Claw
        if (gamepad2.left_trigger > 0.5) {
            robo.claw.setPosition(robo.clawclosed);
            robo.intakeclaw.setPosition(robo.intakeopen);
        } else if (gamepad2.right_trigger > 0.5) {
            robo.claw.setPosition(robo.clawopenfull);
            robo.intakeclaw.setPosition(robo.intakeclosed);
        }
        //endregion
        //endregion

        //region - Intake
        //region - Extendo
        if (gamepad2.dpad_right) {
            robo.inExtR.setPosition(robo.inExtR.getPosition() + 0.002);
        } else if (gamepad2.dpad_left) {
            robo.inExtR.setPosition(robo.inExtR.getPosition() - 0.002);
        }

        if (gamepad2.dpad_up) {
            robo.inExtL.setPosition(robo.inExtL.getPosition() + 0.002);
        } else if (gamepad2.dpad_down) {
            robo.inExtL.setPosition(robo.inExtL.getPosition() - 0.002);
        }
        //endregion

        //region - Intake Pivot
        if (gamepad1.x) {
            robo.intakepivot.setPosition(robo.intakepivot.getPosition() - 0.002);
        } else if (gamepad1.b) {
            robo.intakepivot.setPosition(robo.intakepivot.getPosition() + 0.002);
        }
        //endregion

        //region - Intake Wrist
        if ((gamepad1.left_trigger > 0.75) && (gamepad1.right_trigger < 0.1)) {
            robo.intakewrist.setPosition(robo.intakewrist.getPosition() + 0.002);
        } else if ((gamepad1.right_trigger > 0.75) && (gamepad1.left_trigger < 0.1)) {
            robo.intakewrist.setPosition(robo.intakewrist.getPosition() - 0.002);
        }
        //endregion

        //region - Intake Claw
        if (gamepad2.x) {
            robo.intakeclaw.setPosition(robo.intakeclaw.getPosition() + 0.002);
        } else if (gamepad2.b) {
            robo.intakeclaw.setPosition(robo.intakeclaw.getPosition() - 0.002);
        }
        //endregion
        //endregion

        //Use this as a "master set position" if need to replace servos or add a button to do just one if you need
        //region - Intake: in, closed, down, 0 wrist & Outtake: arm straight up, claw rotate up, claw closed
        if (gamepad1.dpad_right) {
            robo.inExtL.setPosition(robo.intakeleftin);
            robo.inExtR.setPosition(robo.intakerightin);
            robo.intakepivot.setPosition(robo.intakepivotdown);
            robo.arml.setPosition(robo.armlup);
            robo.armr.setPosition(robo.armrup);
            robo.clawrotate.setPosition(robo.clawflat);
            robo.claw.setPosition(robo.clawclosed);
            robo.intakewrist.setPosition(robo.intakewrist0);
            robo.intakeclaw.setPosition(robo.intakeclosed);
        }
        //endregion

        telemetry.addData("Lift Mag: ", robo.liftMag.getState());
        telemetry.addData("Beam State: ", robo.clawbeam.getState());
        telemetry.addData("Lift Pos: ", robo.lift1.getCurrentPosition());
        telemetry.addData("Arm L Pos: ", robo.arml.getPosition());
        telemetry.addData("Arm R Pos: ", robo.armr.getPosition());
        telemetry.addData("Claw Rotate Pos: ", robo.clawrotate.getPosition());
        telemetry.addData("Claw: ", robo.claw.getPosition());

        telemetry.addData("Intake Claw Pos: ", robo.intakeclaw.getPosition());
        telemetry.addData("Intake Pivot Pos: ", robo.intakepivot.getPosition());
        telemetry.addData("In Ext L Pos: ", robo.inExtL.getPosition());
        telemetry.addData("In Ext R Pos: ", robo.inExtR.getPosition());
        telemetry.addData("Intake Wrist Pos: ", robo.intakewrist.getPosition());

        telemetry.addData("Test Servo 1 Pos: ", testservo1pos);
        telemetry.addData("Test Servo 2 Pos: ", testservo2pos);


        telemetry.update();
    }
}
