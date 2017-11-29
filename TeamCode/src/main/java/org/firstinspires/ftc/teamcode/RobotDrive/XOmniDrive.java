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

    public void init(HardwareMap hardwareMap){
        FL = hardwareMap.dcMotor.get("fl");
        FR = hardwareMap.dcMotor.get("fr");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
    }

    //Tele

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
}
