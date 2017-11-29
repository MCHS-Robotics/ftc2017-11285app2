package org.firstinspires.ftc.teamcode.RobotDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class XOmniDrive implements MoveableRobot{

    DcMotor FL,FR,BR,BL,liftP;
    float y,x2,x,deadZone,turnPower,movePower;

    double Dr;
    double Cr;
    double Dw;
    double Cw;
    int Mstep;

    //Creation

     /**
    * Creates a basic x-omni drive for the robot with set defaults
    * @param    hardwaremap the hardware map of the robot for initialization
    */
    public XOmniDrive(HardwareMap hardwareMap){
        deadZone = 0;
        turnPower = .5f;
        movePower = .5f;

        Dr = 19;
        Cr = Math.PI * Dr;
        Dw = 4;
        Cw = Math.PI * Dw;
        Mstep = 1120;

        init(hardwareMap);
    }

     /**
    * Creates a x-omni drive for the robot with variables for the teleop
    * @param    deadZone    the radius (from 0-1) of the deadzone from the gamepad
    * @param    turnPower   scaling modifier for the turning power of the robot
    * @param    movePower   scaling modifier for the translation movement power of the robot
    * @param    hardwaremap the hardware map of the robot for initialization
    */
    public XOmniDrive(float deadZone, float turnPower, float movePower, HardwareMap hardwareMap){
        this.deadZone = deadZone;
        this.turnPower = turnPower;
        this.movePower = movePower;

        Dr = 19;
        Cr = Math.PI * Dr;
        Dw = 4;
        Cw = Math.PI * Dw;
        Mstep = 1120;

        init(hardwareMap);
    }

      /**
    * Creates a x-omni drive for the robot with variables for the autonomous
    * @param    robotDiameter   the length, in inches, between 2 diagonal wheels 
    * @param    wheelDiameter   the diameter, in inches, of the wheels used on the robot
    * @param    motorControllerSteps    the amount of ticks the motors have in one revolution
    * @param    hardwaremap the hardware map of the robot for initialization
    */
    public XOmniDrive(double robotDiameter, double wheelDiameter, int motorControllerSteps, HardwareMap hardwareMap){
        deadZone = 0;
        turnPower = .5f;
        movePower = .5f;

        Dr = robotDiameter;
        Cr = Math.PI * Dr;
        Dw = wheelDiameter;
        Cw = Math.PI * Dw;
        Mstep = motorControllerSteps;

        init(hardwareMap);
    }

       /**
    * Initializes the motors for the robot
    * @param    hardwaremap the hardware map of the robot for initialization
    */
    public void init(HardwareMap hardwareMap){
        FL = hardwareMap.dcMotor.get("fl");
        FR = hardwareMap.dcMotor.get("fr");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
    }

    //Tele

     /**
    * moves the robot in teleop using the 2 thumbsticks
    * Left joystick: movement in any direction
    * Right joystick: turning
    * @param    gamepad1 the first controller being used for the robot
    * @param    gamepad2 the second controller being used for the robot
    */
    public void run(Gamepad gamepad1, Gamepad gamepad2){
        x = gamepad1.left_stick_x;
        if(Math.abs(x) < deadZone)x = 0;
        y = -gamepad1.left_stick_y;
        if(Math.abs(y) < deadZone)y = 0;
        x2 = gamepad1.right_stick_x;
        if(Math.abs(x2) < deadZone)x2 = 0;
        FL.setPower((x+y) * movePower + x2 * turnPower);
        FR.setPower((-x+y) * movePower - x2 * turnPower);
        BL.setPower((-x+y) * movePower + x2 * turnPower);
        BR.setPower((x+y) * movePower - x2 * turnPower);
    }

    //Auto

      /**
    * moves the robot forward a variable amount of inches
    * @param    inches the distance, in inches, the robot moves
    */
    public void forward(float inches){
        int pos = (int)(inches * Mstep * Math.sqrt(2) / Cw);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setTargetPosition(pos);
        FL.setPower(0.3);
        FR.setPower(0.3);
        BL.setPower(0.3);
        BR.setPower(0.3);
        while(FL.isBusy()){
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

       /**
    * moves the robot backward a variable amount of inches
    * @param    inches the distance, in inches, the robot moves
    */
    public void backward(float inches){
        int pos = -(int)(inches * Mstep * Math.sqrt(2) / Cw);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setTargetPosition(pos);
        FL.setPower(0.3);
        FR.setPower(-0.3);
        BL.setPower(-0.3);
        BR.setPower(-0.3);
        while(FL.isBusy()){
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

        /**
    * moves the robot left a variable amount of inches
    * @param    inches the distance, in inches, the robot moves
    */
    public void left(float inches) {
        int pos = (int)(inches * Mstep * Math.sqrt(2) / Cw);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setTargetPosition(pos);
        FL.setPower(0.3);
        FR.setPower(-0.3);
        BL.setPower(-0.3);
        BR.setPower(0.3);
        while(FL.isBusy()){
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

          /**
    * moves the robot right a variable amount of inches
    * @param    inches the distance, in inches, the robot moves
    */
    public void right(float inches){
        int pos = -(int)(inches * Mstep * Math.sqrt(2) / Cw);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setTargetPosition(pos);
        FL.setPower(0.3);
        FR.setPower(0.3);
        BL.setPower(0.3);
        BR.setPower(-0.3);
        while(FL.isBusy()){
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

       /**
    * turns the robot clockwise a variable amount of degrees
    * @param    degrees the amount of degrees the robot turns
    */
    public void clockwise(int degrees){
        int pos = (int)(degrees * Cr * Mstep / 360 / Cw);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setTargetPosition(pos);
        FL.setPower(0.3);
        FR.setPower(-0.3);
        BL.setPower(0.3);
        BR.setPower(-0.3);
        while(FL.isBusy()){}
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

       /**
    * turns the robot counter clockwise a variable amount of degrees
    * @param    degrees the amount of degrees the robot turns
    */
    public void cClockwise(int degrees){
        int pos = -(int)(degrees * Cr * Mstep / 360 / Cw);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setTargetPosition(pos);
        FL.setPower(0.3);
        FR.setPower(0.3);
        BL.setPower(-0.3);
        BR.setPower(0.3);
        while(FL.isBusy()){}
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    /**
    * tests robot to make sure it is in working order by rotating each wheel forward for a variable amt of time
    * @param    seconds time spent moving motors
    */
    public void testDrive(int seconds){
        FL.setPower(0.5);
        sleep(seconds*1000);
        FL.setPower(0);
        FR.setPower(0.5);
        sleep(seconds*1000);
        FR.setPower(0);
        BR.setPower(0.5);
        sleep(seconds*1000);
        BR.setPower(0);
        BL.setPower(0.5);
        sleep(seconds*1000);
        BL.setPower(0);
    }
}
