package org.firstinspires.ftc.teamcode;


import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous

public class ColorSensorTester extends LinearOpMode {

    // todo: write your code here
    ColorSensor sensorColor;
    public void runOpMode()
    {
         sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");
         float hsvValues[] = {0F, 0F, 0F};
         final float values[] = hsvValues;
         final double SCALE_FACTOR = 255;
         int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
         final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
         waitForStart();
         while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            // telemetry.addData("Alpha", sensorColor.alpha());
            // telemetry.addData("Red  ", sensorColor.red());
            // telemetry.addData("Green", sensorColor.green());
            // telemetry.addData("Blue ", sensorColor.blue());
            // telemetry.addData("Hue", hsvValues[0]);
            if(hsvValues[0] > 100 && hsvValues[0] < 319)
        {
            telemetry.addData("Blue Ball Detected ", hsvValues[0]);
            telemetry.update();
            // ballDetected = "blue";
        } else if (hsvValues[0] > 327) {
            telemetry.addData("Red Ball Detected ", hsvValues[0]);
            telemetry.update();
            // ballDetected = "red";
        } else {
            telemetry.addData("No Ball Detected ", hsvValues[0]);
            telemetry.update();
        }
        
            telemetry.update();
         }

    }
}
