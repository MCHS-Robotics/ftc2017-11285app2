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
package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class VuforiaIsDaWae implements Runnable{

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackable;
    Telemetry telemetry;
    boolean run = true;
    /**
     * Creates a VuforiaIsDaWae object
     * @param hardwareMap The hardware map of the robot
     */
   public VuforiaIsDaWae(HardwareMap hardwareMap,Telemetry telemetry) {

       this.telemetry = telemetry;
       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
       parameters.cameraMonitorFeedback= VuforiaLocalizer.Parameters.CameraMonitorFeedback.BUILDINGS;


       parameters.vuforiaLicenseKey = "Acbhfjv/////AAAAGQwVGIAmv0V3nNH7nrtPGJVSuirk278Uo+j+394hRfZGLNmucewzhlA4ux8ZUQz0OpL1mJuPW+lKbjippfmpGiiXsvpnm2p6prlsNHzZNthjThZyFArOkXDkjL5bYlVT3zlo3oc+XNg/W4rBdXHeBpw6OgO1a7D0xhGHkqaihUWqeESvWtcH+uSJXha/umtRGu0DIPRW0n4a3Z0cjbNzhicHvrGOWuwuVhyua1IcrOqed3/bTdts+JhuG3TakwFBb1GpIkWXNziiorUUXVstPVNVrty3AnesNZ11gsJyHVu3YCKi//itjkqr/6xmINr4LDrwdf1Pg2e9L9iT9qtFWjrrnEQYGzxgpnrj0+PvzWWw";


       parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
       this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

       relicTrackable = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
   }

    /**
     * Activates the relicTrackables for vueforia
     */
   public void activate(){
       run = true;
       relicTrackable.activate();
   }

    /**
     * Deactivates the relicTrackables for vueforia
     */
    public void deactivate(){
        relicTrackable.deactivate();
        run = false;
    }

    /**
     * Gets which template is up
     * @return 0 = none, 1 = left, 2 = right, 3 = center
     */
   public int getPos(){
       RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTrackable.get(0));
       if (vuMark == RelicRecoveryVuMark.CENTER) {
           return 3;
       }
       if (vuMark == RelicRecoveryVuMark.LEFT) {
           return 1;
       }
       if (vuMark == RelicRecoveryVuMark.RIGHT) {
           return 2;
       }
       return 0;
   }

    /**
     * Shows which markers are visible
     * @param telemetry
     */
   public void getTelemetryData(Telemetry telemetry) {

       RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTrackable.get(0));
       if (vuMark == RelicRecoveryVuMark.CENTER) {
           telemetry.addData("VueMark", "Center Visible");
       } else {

           telemetry.addData("VueMark", "Center not Visible");
       }
       if (vuMark == RelicRecoveryVuMark.LEFT) {
           telemetry.addLine();
           telemetry.addData("VueMark", "Left Visible");
       } else {
           telemetry.addLine();
           telemetry.addData("VueMark", "Left not Visible");
       }
       if (vuMark == RelicRecoveryVuMark.RIGHT) {
           telemetry.addLine();
           telemetry.addData("VueMark", "Right Visible");
       } else {
           telemetry.addLine();
           telemetry.addData("VueMark", "Right not Visible");
       }
       telemetry.update();
   }

   public void run(){
       while (run)
       getTelemetryData(telemetry);
   }
 }

