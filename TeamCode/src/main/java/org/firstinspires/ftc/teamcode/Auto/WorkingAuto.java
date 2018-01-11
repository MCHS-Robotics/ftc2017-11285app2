/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotDrive.MoveableRobot;
import org.firstinspires.ftc.teamcode.RobotDrive.XOmniDrive;


@Autonomous(name="Kaleb Auto ", group="Auto")
public class WorkingAuto extends LinearOpMode {
    MoveableRobot robot;
    Servo jewel;
    Servo liftL,liftR;
    ColorSensor colorSensor;

    final float[] posL = {.0f,.32f},posR = {1,.6f},posJ = {0,.47f};

    /**
     * Runs a basic autonomous
     */
    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.colorSensor.get("color");
        robot = new XOmniDrive(19.9,4,1120,hardwareMap);
        jewel = hardwareMap.servo.get("jewel");
        jewel.setPosition(0);

        liftL = hardwareMap.servo.get("liftL");
        liftR = hardwareMap.servo.get("liftR");
        liftL.setPosition(posL[1]);
        liftR.setPosition(posR[1]);
        colorSensor.enableLed(true);
        waitForStart();
        ///////////////////////
        while(opModeIsActive())
      colorStats();
        ///////////////////////
        }

    /**
     * Knocks off the left or right jewel
     * @param left  if true knocks off the left jewel else right
     */
    public void moveJewel(boolean left){
            jewel.setPosition(1);
            sleep(100);
            if(left){
                robot.cClockwise(10);
                robot.clockwise(10);
            }else{
                robot.clockwise(10);
                robot.cClockwise(10);
            }
            jewel.setPosition(0);
            sleep(100);
    }

    /**
     * returns RGB color for colorSensor
     */
    public void colorStats(){
        telemetry.addData("Blue:",colorSensor.blue());
        telemetry.addLine();
        telemetry.addData("Green:",colorSensor.green());
        telemetry.addLine();
        telemetry.addData("Red:",colorSensor.red());
        telemetry.update();
    }

    /**
     * returns if the object is red
     * @return true if object is more red then blue
     */
    public boolean isRed(){
        colorSensor.enableLed(true);
        boolean isRed = colorSensor.red() > colorSensor.blue();
        colorSensor.enableLed(false);
        return isRed;
    }

}