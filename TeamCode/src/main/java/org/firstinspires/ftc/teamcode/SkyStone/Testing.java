package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Autonomous(name = "testing", group = "Autonomous")
public class Testing extends LinearOpMode {
    private ColorSensor colorSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor  = hardwareMap.colorSensor.get("color");
        while (opModeIsActive()){
            colorSensor  = hardwareMap.colorSensor.get("color");
            readColorSensor();
        }
    }
    public void readColorSensor(){
        colorSensor.enableLed(true);
        colorSensor.alpha();
        telemetry.addData("LightVal", colorSensor.alpha());
        telemetry.update();
    }
}
