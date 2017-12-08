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
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotDrive.MoveableRobot;
import org.firstinspires.ftc.teamcode.RobotDrive.XOmniDrive;


@Autonomous(name="Kaleb Auto ", group="Auto")
public class WorkingAuto extends LinearOpMode {
    static final int LED_CHANNEL = 5;
    MoveableRobot robot;
    Servo jewel;
    ColorSensor sensorRGB;
    static DeviceInterfaceModule cdim;
    @Override
    public void runOpMode() {
        robot = new XOmniDrive(19.9,4,1120,hardwareMap);
        jewel = hardwareMap.servo.get("jewel");
        jewel.setPosition(0);
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        sensorRGB = hardwareMap.colorSensor.get("sensor_color");
        cdim.setDigitalChannelState(LED_CHANNEL, false);
        waitForStart();
        ///////////////////////

        ///////////////////////
        }

    /**
     * knocks of the jewel when the robot is directly in front of it
     * @param   left    if true turns left otherwise it turns right
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
     * checks to see if color detected by color sensor is more red than blue
     * @return  boolean that is true if color is more red than blue
     * */
    public boolean isRed(){
        //soundPlayer.play(hardwareMap.appContext,0);
        cdim.setDigitalChannelState(LED_CHANNEL, true);//turns on the led
        try {
            Thread.sleep(100);//sleeps the robot for 100 milliseconds
        }catch(Exception e){

        }
        boolean isR = sensorRGB.red()>sensorRGB.blue();//checks if the red detected is more than the blue detected
        cdim.setDigitalChannelState(LED_CHANNEL, false);//turns off led
        return isR;//returns boolean

    }

}