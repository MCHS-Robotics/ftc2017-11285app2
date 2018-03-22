package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by student on 3/21/18.
 */

public class Arm {
    DcMotor elbow;
    Servo wrist,hand;
    float[] handPos = {0,1}; //{open,closed}
    float elbowPos = 1000; //maximum movement
    float wristPos = .5f; //resting
    public Arm(HardwareMap map){
        elbow = map.dcMotor.get("elbow");
        wrist = map.servo.get("wrist");
        hand = map.servo.get("hand");
    }

    public void openHand(){
        hand.setPosition(handPos[0]);
    }

    public void closeHand(){
        hand.setPosition(handPos[1]);
    }

    public void setElbowPower(float power){
        elbow.setPower(power);
    }

    public void runElbowTo(int pos,float power){
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setTargetPosition(pos);
        elbow.setPower(power);
    }

    public void checkElbow(){
        if(!elbow.isBusy()){
            elbow.setPower(0);
        }
    }

    public void checkElbow(int maxError){

    }
}
