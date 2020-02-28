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
            release.setPosition(0);
            initAttachCode();
            forward(50, .5);
            stopMotors();
            sideRight(4400, .5 );
            forward(90, .5);
            gyroPivot(1, 95);
            forward(60, .5);
            sideRight(1000, 5.);
            stopMotors();
            clamp1.setPosition(1);
            clamp2.setPosition(1);
            break;
        }
    }
}
