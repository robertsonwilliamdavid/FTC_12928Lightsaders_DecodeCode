package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

//@Disabled
@TeleOp(name = "BrushLab Color Test", group = "Opmode")
public class Deep_BrushLabColor_Test extends OpMode {

    DeepRoboConstants robo = new DeepRoboConstants();

    @Override
    public void init() {
        robo.pin0 = hardwareMap.digitalChannel.get("color0");
        robo.pin1 = hardwareMap.digitalChannel.get("color1");

        telemetry.addData("Ready", "");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (robo.pin0.getState() && robo.pin1.getState()) {
            robo.colorseen = "Yellow";
        } else if (robo.pin0.getState()) {
            robo.colorseen = "Blue";
        } else if (robo.pin1.getState()) {
            robo.colorseen = "Red";
        } else {
            robo.colorseen = "None";
        }

        telemetry.addData("Color seen: ", robo.colorseen);
        telemetry.update();
    }
}
