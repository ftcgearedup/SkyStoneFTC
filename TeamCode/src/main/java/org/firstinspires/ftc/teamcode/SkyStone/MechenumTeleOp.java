package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
public class MechenumTeleOp extends SkystoneRobot {

    public void initTeleop(){

        // set wheel power variables
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // set attachment power variables
      //  intake.setPower(0);
        lift.setPower(0);

        intake.setPosition(0.2);
        // set deadzone
        gamepad1.setJoystickDeadzone(0.2f);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initDriveHardware();
        initAttachCode();
        initTeleop();
        waitForStart();
        while (opModeIsActive()){
            move();

        }
    }

    public void move() {
        //the left Joystick controls movement, and the right controls turning.
        //in order to counteract the differences between the basic formula for strafing and the one for moving forwards and backwards,
        //we use trigonometry to derive relevant powers for each wheel tio move at the correct angles.
        // translating polar coordanates of joystick to polar coordinates of mechenum drive
        double r = Math.hypot(gamepad1.right_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_x;
        final double fr = r * Math.cos(robotAngle) + rightX;
        final double fl = r * Math.sin(robotAngle) - rightX;
        final double bl = r * Math.sin(robotAngle) + rightX;
        final double br = r * Math.cos(robotAngle) - rightX;

        double intakePower = 0.0;

        if (gamepad1.left_trigger > 0) {
            frontRight.setPower(fr / 2);
            frontLeft.setPower(fl / 2);
            backLeft.setPower(bl / 2);
            backRight.setPower(br / 2);
        } else if (gamepad1.right_trigger > 0) {
            frontRight.setPower(fr / 4);
            frontLeft.setPower(fl / 4);
            backLeft.setPower(bl / 4);
            backRight.setPower(br / 4);
        } else {
            frontRight.setPower(fr);
            frontLeft.setPower(fl);
            backLeft.setPower(bl);
            backRight.setPower(br);
        }

        //clamps
        if (gamepad1.left_bumper){
            clamp1.setPosition(1);
            clamp2.setPosition(0);
        }else{
            clamp1.setPosition(0);//Was 1, changed to .8 so the servo does not push on the frame.
            clamp2.setPosition(1);
        }


        //GAMEPAD 2


        //Dropper
        if (gamepad2.b) {
            telemetry.addData("a Button", "pressed");
            release.setPosition(0);
            telemetry.update();
        } else {
           release.setPosition(.6);
            telemetry.addData("a Button", " not pressed");
            telemetry.update();
        }
        //intake
        while (gamepad2.x){
            telemetry.addData("Intake", "closed");
            intake.setPosition(1);
            telemetry.update();
       }
        while (!gamepad2.x){
            intake.setPosition(0);
        }
        //Telemetry
        telemetry.addData("motor speeds", "fl " + fl + " fr " + fr + " bl " + bl + " br " + br);
        telemetry.update();

    }
}