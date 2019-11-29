package org.firstinspires.ftc.teamcode.SkyStone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaSkyStoneNavigationWebcam;
@Autonomous(name = "MainAuto", group = "Autonomous")
public class AutoBasic extends VuforiaSkyStoneNavigationWebcam {
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor frontLeft;

    //Attachments
    private DcMotor intakeLeft;
    private DcMotor intakeRight;
   //private DcMotor lift;
  //  private Servo dropper;
    //private Servo clamp1;
   // private Servo clamp2;



    private double ticksPerRevNR20 = 560;
    private double ticksPerRevNR40 = 1120;
    private double ticksPerRevNR60 = 1680;

    //The post gear box gear ratio.
    private double gearRatio = 1.0;
    //The circumference of the drive wheel.
    private double wheelCircumference = 31.9024; // ??
    //Formula to calculate ticks per centimeter for the current drive set up.FORWARDS/BACKWARD ONLY
    private double ticksPerCm = (ticksPerRevNR40 * gearRatio) / wheelCircumference;
    //Formula to calculate ticks per centimeter for the current drive set up.SIDEWAYS

    public void initHardware() {
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
      //  clamp1 = hardwareMap.servo.get("clamp1");
        //clamp2 = hardwareMap.servo.cast("clamp2");

        // set wheel direction
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //set attatchment direction
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
      //  lift.setDirection(DcMotorSimple.Direction.FORWARD);

        // set wheel power variables
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // set attachment power variables
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
      //  lift.setPower(0);

    }

    public void runOpMode(){
        waitForStart();
       while (opModeIsActive()) {
           initHardware();
           setZeroPowBehv(DcMotor.ZeroPowerBehavior.BRAKE);
           setDirection();
           setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

           chooseprogram();
       }
    }

    public void setZeroPowBehv(DcMotor.ZeroPowerBehavior behv) {
        frontLeft.setZeroPowerBehavior(behv);
        frontRight.setZeroPowerBehavior(behv);
        backLeft.setZeroPowerBehavior(behv);
        backRight.setZeroPowerBehavior(behv);
    }

    public void setDirection() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    //for backwards use negative power
    //Pass in centimeters.
    public void forward(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double targetDistanceTicks = Math.abs(targetDistance * ticksPerCm);
        double currentDistanceTicks = 0;
        while ((Math.abs(currentDistanceTicks) < targetDistanceTicks) && opModeIsActive()) {
            telemetry.addData("Target pos ticks: ", targetDistanceTicks);
            telemetry.addData("Target Distance:", targetDistance + "cm");
            currentDistanceTicks = (frontRight.getCurrentPosition() +
                    frontLeft.getCurrentPosition() +
                    backRight.getCurrentPosition() +
                    backLeft.getCurrentPosition()) / 4.0;
            telemetry.addData("Current pos ticks Avg: ", currentDistanceTicks);
            telemetry.addData("Current Distance cm", currentDistanceTicks / ticksPerCm);
            telemetry.update();

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }
        stopMotors();
    }

    public void chooseprogram(){
       telemetry.addData("ChoosePosition", "RED1-A, RED2-B, Blue1-X, Blue2-Y");
        if(gamepad1.a){
            telemetry.addData("RobotPosition","RED1");
            red1();
            telemetry.update();
        } else if (gamepad1.b){
            telemetry.addData("RobotPosition","RED2");
            red2();
            telemetry.update();
        } else if (gamepad1.x){
            telemetry.addData("RobotPosition", "BLUE1");
            blue1();
            telemetry.update();
        } else if (gamepad1.y){
            telemetry.addData("RobotPosition","BLUE2");
            blue2();
            telemetry.update();
        }
    }

    public void red1(){
        waitForStart();
       if (opModeIsActive()) {
            forward(65, .5);
            double encoderVal = frontLeft.getCurrentPosition();
           /* while (lastSkyStoneLocation == null) {
                setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //driveLeft
                frontLeft.setPower(-.25);
                frontRight.setPower(.25);
                backLeft.setPower(.25);
                backRight.setPower(-.25);
                encoderVal = frontLeft.getCurrentPosition();
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
*/
           sideLeft(1000,.5);
            intake(1);
            forward(30, .5);
            forward(30, -.5);
            forward(0,0);
            sideLeft(500, .5);
            forward(60,.5);
            forward(60, -5);
            intake(0);
          /*  sideRight(10,.5);

            setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (frontLeft.getCurrentPosition() >= -encoderVal) {
                frontLeft.setPower(-.5);
                frontRight.setPower(.5);
                backLeft.setPower(.5);
                backRight.setPower(-.5);
            }
            */
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
          //  sideRight(4000, 1);
          //  clamp1.setPosition(1);
        //    clamp2.setPosition(1);
            //forward(30, 1);
         //   clamp1.setPosition(0);
         //   clamp2.setPosition(0);
            //sideLeft(4000, 1);
            //pivotCC(90, 1);
         //   clamp1.setPosition(1);
         //   clamp2.setPosition(1);
            //sideRight(60, 1);
            //intake(-1);

        }
    }
    public void red2(){
        waitForStart();
        while (opModeIsActive()){
            forward(30, 1);
            setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double encoderVal = frontLeft.getCurrentPosition();
            while (lastSkyStoneLocation == null){
                //driveLeft
                frontLeft.setPower(.5);
                frontRight.setPower(-.5);
                backLeft.setPower(-.5);
                backRight.setPower(.5);
                encoderVal = frontLeft.getCurrentPosition();
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            intake(1);
            forward(10,1);
            forward(10,-1);
            intake(0);
            setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while(frontLeft.getCurrentPosition() >= -encoderVal){
                frontLeft.setPower(-.5);
                frontRight.setPower(.5);
                backLeft.setPower(.5);
                backRight.setPower(-.5);
            } frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            sideRight(30, 1);
           // clamp1.setPosition(1);
            //clamp2.setPosition(1);
            forward(30, 1);
            //clamp1.setPosition(0);
            //clamp2.setPosition(0);
            sideLeft(30,1);
            pivotCC(90, 1);
            //clamp1.setPosition(1);
            //clamp2.setPosition(1);
            sideRight(60,1);
            intake(-1);
        }
    }
    public void blue1(){
        waitForStart();
        while (opModeIsActive()) {
            forward(30, 1);
            setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double encoderVal = frontLeft.getCurrentPosition();
            while (lastSkyStoneLocation == null) {
                //driveLeft
                frontLeft.setPower(-.5);
                frontRight.setPower(.5);
                backLeft.setPower(.5);
                backRight.setPower(-.5);
                encoderVal = frontLeft.getCurrentPosition();
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            intake(1);
            forward(10, 1);
            forward(10, -1);
            intake(0);
            setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (frontLeft.getCurrentPosition() >= -encoderVal) {
                frontLeft.setPower(.5);
                frontRight.setPower(-.5);
                backLeft.setPower(-.5);
                backRight.setPower(.5);
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            sideLeft(90, 1);
           // clamp1.setPosition(1);
            //clamp2.setPosition(1);
            forward(30, 1);
            //clamp1.setPosition(0);
            //clamp2.setPosition(0);
            sideRight(30, 1);
            pivotCC(90, 1);
            //clamp1.setPosition(1);
            //clamp2.setPosition(1);
            sideLeft(60, 1);
            intake(-1);
        }
    }
    public void blue2(){
        waitForStart();
        while (opModeIsActive()){
            forward(30, 1);
            setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double encoderVal = frontLeft.getCurrentPosition();
            while (lastSkyStoneLocation == null){
                //driveLeft
                frontLeft.setPower(-.5);
                frontRight.setPower(.5);
                backLeft.setPower(.5);
                backRight.setPower(-.5);
                encoderVal = frontLeft.getCurrentPosition();
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            intake(1);
            forward(10,1);
            forward(10,-1);
            intake(0);
            setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while(frontLeft.getCurrentPosition() >= -encoderVal){
                frontLeft.setPower(.5);
                frontRight.setPower(-.5);
                backLeft.setPower(-.5);
                backRight.setPower(.5);
            } frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            sideLeft(30, 1);
            //clamp1.setPosition(1);
            //clamp2.setPosition(1);
            forward(30, 1);
            //clamp1.setPosition(0);
            //clamp2.setPosition(0);
            sideRight(30,1);
            pivotCC(90, 1);
            //clamp1.setPosition(1);
            //clamp2.setPosition(1);
            sideLeft(60,1);
            intake(-1);
        }
    }
    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void sideRight(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double currentDistance = 0;
        while ((currentDistance < targetDistance) && opModeIsActive()) {
            currentDistance = frontLeft.getCurrentPosition();
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }

    }

    public void sideLeft(double targetDistance, double power) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double currentDistance = 0;
        while (currentDistance < targetDistance && opModeIsActive()) {
            currentDistance = frontRight.getCurrentPosition();
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }
    }
    public void intake(double power){
        intakeLeft.setPower(power);
        intakeRight.setPower(power);
    }





    //clockwise is 0 cc is 1
    public void pivotCW(double degree, double power) {

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentDegree = 0;
        while ((currentDegree < degree) && opModeIsActive()) {
            currentDegree = (frontLeft.getCurrentPosition() + backLeft.getCurrentPosition())/2;
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        }
        stopMotors();
    }

    public void pivotCC(double degree, double power) {

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentDegree = 0;
        while (currentDegree < degree && opModeIsActive()) {
            currentDegree = (frontRight.getCurrentPosition() + backRight.getCurrentPosition())/2;
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }
        stopMotors();
    }
}
