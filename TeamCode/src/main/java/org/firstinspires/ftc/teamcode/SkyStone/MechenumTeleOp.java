package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
public class MechenumTeleOp extends OpMode {
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;

    //Attachments
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
  //  private DcMotor lift;
   // private Servo dropper;

    @Override
    public void init() {
        // init the motors
        frontRight = hardwareMap.dcMotor.get("fr");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        backLeft = hardwareMap.dcMotor.get("bl");

        //init the Attachments
        intakeLeft = hardwareMap.dcMotor.get("il");
        intakeRight = hardwareMap.dcMotor.get("ir");
      //  lift = hardwareMap.dcMotor.get("lift");
      //  dropper = hardwareMap.servo.get("drop");

        // set wheel direction
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //set attatchment direction
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
       // lift.setDirection(DcMotorSimple.Direction.FORWARD);

        // set wheel power variables
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // set attachment power variables
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        //lift.setPower(0);

        // set deadzone
        gamepad1.setJoystickDeadzone(0.1f);
    }

    public void move() {
        //the left Joystick controls movement, and the right controls turning.
        //in order to counteract the differences between the basic formula for strafing and the one for moving forwards and backwards,
        //we use trigonometry to derive relevant powers for each wheel tio move at the correct angles.
        // translating polar coordanates of joystick to polar coordinates of mechenum drive
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
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
        //INTAKE
        if (gamepad2.dpad_up) {
            intakeRight.setPower(1);
            intakeLeft.setPower(1);
        } else if (gamepad2.dpad_down) {
            intakeRight.setPower(-1);
            intakeLeft.setPower(-1);
        } else if (gamepad2.dpad_right) {
            intakeRight.setPower(1);
            intakeLeft.setPower(0);
        } else if (gamepad2.dpad_left) {
            intakeLeft.setPower(1);
            intakeRight.setPower(0);
        } else {
            intakeRight.setPower(0);
            intakeLeft.setPower(0);
        }
        //Dropper
    //    if (gamepad2.right_trigger > 0) {
     //       dropper.setPosition(1);
   //     } else {
     //       dropper.setPosition(0);
     //   }
        //lift
      //  lift.setPower(gamepad2.right_stick_y);
        //Telemetry
        telemetry.addData("motor speeds", "fl " + fl + " fr " + fr + " bl " + bl + " br " + br);
        telemetry.update();
    }

    @Override
    public void loop() {
        init();
        move();
    }

}