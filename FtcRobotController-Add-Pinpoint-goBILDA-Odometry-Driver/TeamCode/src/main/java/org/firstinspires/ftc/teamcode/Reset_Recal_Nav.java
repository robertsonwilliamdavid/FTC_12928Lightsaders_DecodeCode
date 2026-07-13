package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

public class Reset_Recal_Nav {

    public void resetnavigation(SparkFunOTOS OTOS, GoBildaPinpointDriver ODO, boolean reset_OTOS, boolean recal_OTOS_IMU, boolean reset_opods, boolean recal_opods) {
        if (reset_OTOS) {
            OTOS.resetTracking();
        }

        if (recal_OTOS_IMU) {
            OTOS.calibrateImu();
        }

        if (reset_opods) {
            ODO.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        }

        if (recal_opods) {
            ODO.recalibrateIMU(); //recalibrates the IMU without resetting position
        }
    }
}
