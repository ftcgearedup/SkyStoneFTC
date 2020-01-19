package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Autonomous(name = "testing", group = "Autonomous")
public class Testing extends SkystoneRobot {
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            forwardWithProportionalDrive(30, .5);
        }
    }
}