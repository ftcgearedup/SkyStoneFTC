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

        // set deadzone
        gamepad1.setJoystickDeadzone(0.2f);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        initDriveHardware();
        initAttachCode();
        initTeleop();
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

        //GAMEPAD 2

        //lift
       if (gamepad2.right_trigger > 0){
           lift.setPower(1);
       } if(gamepad2.left_trigger >0){
           lift.setPower(-1);
        } else{
           lift.setPower(0);
        }
        //Dropper
        if (gamepad2.a) {
            telemetry.addData("a Button", "pressed");
       //     release.setPosition(.1);
            telemetry.update();
        } else {
        //    release.setPosition(.4);
            telemetry.addData("a Button", " not pressed");
            telemetry.update();
        }
        //intake
       // intake.setPower(gamepad2.right_stick_y);
      //  if (gamepad2.dpad_left){
      //      intake.setPower(1);
     //   }
      //  if (gamepad2.dpad_right){
       //     intake.setPower(-1);
     //   }
        //Telemetry
        telemetry.addData("motor speeds", "fl " + fl + " fr " + fr + " bl " + bl + " br " + br);
        telemetry.update();

        //clamps
        if (gamepad2.right_bumper){
            clamp1.setPosition(0);
        }else{
            clamp1.setPosition(.8); //Was 1, changed to .8 so the servo does not push on the frame.
        }

        if(gamepad2.left_bumper){
            clamp2.setPosition(1);
        }else{
            clamp2.setPosition(.4); //Was 0, changed to .2 so the servo does not push on the frame.
        }

    }
}