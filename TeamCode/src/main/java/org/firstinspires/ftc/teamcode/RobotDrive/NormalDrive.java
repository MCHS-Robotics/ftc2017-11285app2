package org.firstinspires.ftc.teamcode.RobotDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NormalDrive implements MoveableRobot{

    DcMotor L,R;
    float y,x2,deadZone,turnPower,movePower;

    double Dr;
    double Cr;
    double Dw;
    double Cw;
    int Mstep;

    //Creation

    public NormalDrive(HardwareMap hardwareMap){
        deadZone = 0;
        turnPower = 1;
        movePower = 1;

        Dr = 19;
        Cr = Math.PI * Dr;
        Dw = 4;
        Cw = Math.PI * Dw;
        Mstep = 1120;

        init(hardwareMap);
    }

    public NormalDrive(float deadZone,float turnPower,float movePower,HardwareMap hardwareMap){
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

    public NormalDrive(double robotDiameter,double wheelDiameter,int motorControllerSteps,HardwareMap hardwareMap){
        deadZone = 0;
        turnPower = 1;
        movePower = 1;

        Dr = robotDiameter;
        Cr = Math.PI * Dr;
        Dw = wheelDiameter;
        Cw = Math.PI * Dw;
        Mstep = motorControllerSteps;

        init(hardwareMap);
    }

    public void init(HardwareMap hardwareMap){
        L = hardwareMap.dcMotor.get("l");
        R = hardwareMap.dcMotor.get("r");
        R.setDirection(DcMotor.Direction.REVERSE);
    }

    //Tele

    public void run(Gamepad gamepad1, Gamepad gamepad2){
        y = gamepad1.left_stick_y;
        if(Math.abs(y) < deadZone)y = 0;
        x2 = gamepad1.right_stick_x;
        if(Math.abs(x2) < deadZone)x2 = 0;
        L.setPower(y * movePower + x2 * turnPower);
        R.setPower(y * movePower - x2 * turnPower);
    }

    //Auto

    public void forward(float inches){
        int pos = (int)(inches * Mstep / Cw);
        L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L.setTargetPosition(pos);
        R.setTargetPosition(pos);
        L.setPower(0.3);
        R.setPower(0.3);
        while(L.isBusy()&&R.isBusy()) {
        }
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void backward(float inches){
        int pos = -(int)(inches * Mstep / Cw);
        L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L.setTargetPosition(pos);
        R.setTargetPosition(pos);
        L.setPower(0.3);
        R.setPower(0.3);
        while(L.isBusy()&&R.isBusy()) {
        }
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void left(float inches){
        cClockwise(90);
        forward(inches);
        clockwise(90);
    }

    public void right(float inches){
        clockwise(90);
        forward(inches);
        cClockwise(90);
    }

    public void clockwise(int degrees){
        int pos = (int)(degrees * Cr * Math.sqrt(2) * Mstep / 360 / Cw);
        L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L.setTargetPosition(pos);
        R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.setTargetPosition(-pos);
        L.setPower(0.3);
        R.setPower(0.3);
        while(L.isBusy() && R.isBusy()){
        }
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void cClockwise(int degrees){
        int pos = (int)(degrees * Cr * Math.sqrt(2) * Mstep / 360 / Cw);
        L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L.setTargetPosition(-pos);
        R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.setTargetPosition(pos);
        L.setPower(0.3);
        R.setPower(0.3);
        while(L.isBusy() && R.isBusy()){
        }
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
