package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Autonomous(name = "testing", group = "Autonomous")
public class Testing extends SkystoneRobot {
   @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            initDriveHardware();
            initAttachCode();
            forward(60, .7);
            stopMotors();
            sideRight(6200, .7 );
            stopMotors();
            clamp1.setPosition(0);
            clamp2.setPosition(0);
            break;
        }
    }
}
