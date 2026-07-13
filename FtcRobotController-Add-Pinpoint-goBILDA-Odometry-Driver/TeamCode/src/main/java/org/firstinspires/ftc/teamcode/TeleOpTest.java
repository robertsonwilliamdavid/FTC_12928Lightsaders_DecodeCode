/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name=" TeleopTest", group="OpMode")
//@Disabled
public class TeleOpTest extends OpMode {

    DeepRoboConstants robo = new DeepRoboConstants();

    private ElapsedTime runtime = new ElapsedTime();

    double x;
    double y;
    double z;

    boolean speed = true;
    double g1ddtime = 0;

    double intakescalar = 1;

    double xoffset = 0.3346;
    double yoffset = 0.9055;
    double linscale = 0.9725;
    double angscale = 0.993;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robo.fl = hardwareMap.get(DcMotor.class, "fl");
        robo.fr = hardwareMap.get(DcMotor.class, "fr");
        robo.bl = hardwareMap.get(DcMotor.class, "bl");
        robo.br = hardwareMap.get(DcMotor.class, "br");

        robo.otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        robo.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        robo.fl.setDirection(DcMotor.Direction.REVERSE);
        robo.bl.setDirection(DcMotor.Direction.REVERSE);

        robo.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robo.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*robo.odo.setOffsets(robo.xodooffset, robo.yodooffset); //these are tuned for 3110-0002-0001 Product Insight #1
        robo.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robo.odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robo.odo.resetPosAndIMU();*/

        robo.fl.setPower(0);
        robo.fr.setPower(0);
        robo.bl.setPower(0);
        robo.br.setPower(0);

        robo.inExtL.setPosition(robo.intakeleftin);
        robo.inExtR.setPosition(robo.intakerightin);

        robo.configureOdo();
        robo.configureOtos();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        runtime.reset();
    }

    public void loop() {
        //use this program for driving and OTOS tuning, use Deep_Axon_Test for servos and lift

        robo.odo.update();

        //region - Set X, Y, & Z to joysticks
        x = gamepad1.right_stick_x;
        y = -gamepad1.left_stick_y;
        z = gamepad1.left_stick_x;

        if (robo.inExtL.getPosition() == robo.intakeleftin) {
            intakescalar = 1;
        } else if ((robo.inExtL.getPosition() == robo.intakeleftout) && (z != 0)) {
            intakescalar = 0.65;
        } else {
            intakescalar = 1;
        }

        if (intakescalar == 0.65) {
            robo.flpower = Range.clip(y + x + z, -1, 1);
            robo.frpower = Range.clip(y - x - z, -1, 1);
            robo.blpower = Range.clip(y + x + z, -1, 1);
            robo.brpower = Range.clip(y - x - z, -1, 1);
        } else {
            robo.flpower = Range.clip(y + x + z, -1, 1);
            robo.frpower = Range.clip(y - x - z, -1, 1);
            robo.blpower = Range.clip(y + x - z, -1, 1);
            robo.brpower = Range.clip(y - x + z, -1, 1);
        }

        //region - Slow/Fast
        if (gamepad1.left_stick_button && ((runtime.milliseconds() - g1ddtime) > 1000)) {
            g1ddtime = runtime.milliseconds();

            if (speed) {
                robo.power = 1;
                speed = false;
            } else {
                robo.power = 0.5;
                speed = true;
            }
        }
        //endregion

        robo.fl.setPower(robo.flpower * robo.power * intakescalar);
        robo.fr.setPower(robo.frpower * robo.power * intakescalar);
        robo.bl.setPower(robo.blpower * robo.power);
        robo.br.setPower(robo.brpower * robo.power);
        //endregion

        //region - Reset OTOS
        if ((gamepad1.right_trigger > 0.5) && (gamepad1.left_trigger > 0.5)) {
            configureOtos(xoffset, yoffset, linscale, angscale);
        }
        //endregion

        //region - Adjust Mount Location of OTOS (right = +x, forward = +y from top of robot)
        if (gamepad1.x) {
            xoffset = xoffset - 0.0005;
        } else if (gamepad1.b) {
            xoffset = xoffset + 0.0005;
        } else if (gamepad1.a) {
            yoffset = yoffset - 0.0005;
        } else if (gamepad1.y) {
            yoffset = yoffset + 0.0005;
        }
        //endregion

        //region - Adjust Scalars for OTOS
        if (gamepad1.dpad_up) {
            linscale = linscale + 0.0005;
        } else if (gamepad1.dpad_down) {
            linscale = linscale - 0.0005;
        } else if (gamepad1.dpad_left) {
            angscale = angscale - 0.0005;
        } else if (gamepad1.dpad_right) {
            angscale = angscale + 0.0005;
        }

        if (gamepad1.start) {
            linscale = 1;
            angscale = 1;
        }
        //endregion

        telemetry.addData("Intake Scalar Is: ", intakescalar);
        telemetry.addData("Speed: ", speed);
        telemetry.addData("Velocity: ", (robo.otos.getVelocity().x + robo.otos.getVelocity().y + robo.otos.getVelocity().h));
        telemetry.addData("OTOS (X, Y, Z): ", robo.otos.getPosition().x + ", " + robo.otos.getPosition().y  +
                ", " + robo.otos.getPosition().h);
        telemetry.addData("X Offset: ", xoffset);
        telemetry.addData("Odo (X, Y, Z): ", robo.odo.getPosition().getX(DistanceUnit.INCH) + ", " +
                robo.odo.getPosition().getY(DistanceUnit.INCH) + ", " + robo.odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("X Offset: ", xoffset);
        telemetry.addData("Y Offset: ", yoffset);
        telemetry.addData("Linear Scalar: ", linscale);
        telemetry.addData("Angular Scalar: ", angscale);
        telemetry.update();

        Log.d("Odo", "Odo (X, Y, Z): " + robo.odo.getPosition().getX(DistanceUnit.INCH) + ", " +
                robo.odo.getPosition().getY(DistanceUnit.INCH) + ", " + robo.odo.getPosition().getHeading(AngleUnit.DEGREES));
    }

    public void configureOtos(double Xoff, double Yoff, double Lscale, double Ascale) {
        robo.otos.setLinearUnit(DistanceUnit.INCH);
        robo.otos.setAngularUnit(AngleUnit.DEGREES);

        // If sensor is 5 inches left (neg X) and 10 inches forward (pos y) of center
        // of robot, and 90 degrees clockwise (neg rotation) from robot's orientation,
        // offset is {-5, 10, -90}
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(Xoff, Yoff, 180);
        robo.otos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        robo.otos.setLinearScalar(Lscale);
        robo.otos.setAngularScalar(Ascale);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        robo.otos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        robo.otos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        robo.otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        robo.otos.getVersionInfo(hwVersion, fwVersion);
    }
}
