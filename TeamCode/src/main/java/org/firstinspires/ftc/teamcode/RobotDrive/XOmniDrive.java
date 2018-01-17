package org.firstinspires.ftc.teamcode.RobotDrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

public class XOmniDrive implements MoveableRobot{
    boolean DEBUGGING = false;
    DcMotor FL,FR,BR,BL,liftP;
    float y,x2,x,deadZone,turnPower,movePower;
    Telemetry telemetry;

    double Dr;
    double Cr;
    double Dw;
    double Cw;
    int Mstep;

    //Creation

     /**
    * Creates a basic x-omni drive for the robot with set defaults
    * @param    hardwareMap the hardware map of the robot for initialization
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
    * @param    hardwareMap the hardware map of the robot for initialization
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
    * @param    hardwareMap the hardware map of the robot for initialization
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
    * @param    hardwareMap the hardware map of the robot for initialization
    */
    public void init(HardwareMap hardwareMap){
        FL = hardwareMap.dcMotor.get("fl");
        FR = hardwareMap.dcMotor.get("fr");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        
        telemetry = null;
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

    /**
     * moves the robot in teleop using the 2 thumbsticks
     * Left joystick: movement in any direction
     * Right joystick: turning
     * @param    gamepad1 the first controller being used for the robot
     * @param    gamepad2 the second controller being used for the robot
     */
    public void runAdvanced(Gamepad gamepad1, Gamepad gamepad2){
        float lookupTable[] = {-1f,-.5f,-.25f,-.125f,0,.125f,.25f,.5f,1f};
        x = gamepad1.left_stick_x;
        if(Math.abs(x) < deadZone)x = 0;
        y = -gamepad1.left_stick_y;
        if(Math.abs(y) < deadZone)y = 0;
        x2 = gamepad1.right_stick_x;
        if(Math.abs(x2) < deadZone)x2 = 0;
        FL.setPower((x+y) * movePower + GetPower(lookupTable,x2));
        FR.setPower((-x+y) * movePower - GetPower(lookupTable,x2));
        BL.setPower((-x+y) * movePower + GetPower(lookupTable,x2));
        BR.setPower((x+y) * movePower - GetPower(lookupTable,x2));
    }

    /**
     * returns a variable length
     * @param numPerSide the data points per side
     * @return the lookup table
     */
    public float[] LookupTable(int numPerSide){
        float[] table = new float[numPerSide* 2 + 1];
        table[numPerSide] = 0;
        for(int i = 0; i < numPerSide;i++){
            table[numPerSide-1-i] = -1.0f*i*i/(numPerSide-1)/(numPerSide-1);
            table[numPerSide+1+i] = 1.0f*i*i/(numPerSide-1)/(numPerSide-1);
        }
        return table;
    }

    /**
     * gets a new power through the lookup table
     * @param lookupTable the lookup table to be used
     * @param data the data point to be matched
     * @return the power from the table
     */
    public float GetPower(float[] lookupTable,float data){
        int len = lookupTable.length/2;
        return lookupTable[Math.round(data * len)+len];
    }

    //Auto

      /**
    * moves the robot forward a variable amount of inches
    * @param    inches the distance, in inches, the robot moves
    */
    public void forward(float inches){
        int pos = (int)(inches * Mstep * Math.sqrt(2) / Cw);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setTargetPosition(pos);
        FR.setTargetPosition(pos);
        BL.setTargetPosition(pos);
        BR.setTargetPosition(pos);
        FL.setPower(0.3);
        FR.setPower(0.3);
        BL.setPower(0.3);
        BR.setPower(0.3);
        while(FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()){
            if(DEBUGGING)displayStats();
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

       /**
    * moves the robot backward a variable amount of inches
    * @param    inches the distance, in inches, the robot moves
    */
    public void backward(float inches){
        int pos = -(int)(inches * Mstep * Math.sqrt(2) / Cw);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setTargetPosition(pos);
        FR.setTargetPosition(pos);
        BL.setTargetPosition(pos);
        BR.setTargetPosition(pos);
        FL.setPower(0.3);
        FR.setPower(0.3);
        BL.setPower(0.3);
        BR.setPower(0.3);
        while(FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()){
            if(DEBUGGING)displayStats();
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

        /**
    * moves the robot left a variable amount of inches
    * @param    inches the distance, in inches, the robot moves
    */
    public void left(float inches) {
        int pos = (int)(inches * Mstep * Math.sqrt(2) / Cw);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setTargetPosition(-pos);
        FR.setTargetPosition(pos);
        BL.setTargetPosition(pos);
        BR.setTargetPosition(-pos);
        FL.setPower(0.3);
        FR.setPower(0.3);
        BL.setPower(0.3);
        BR.setPower(0.3);
        while(FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()){
            if(DEBUGGING)displayStats();
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

          /**
    * moves the robot right a variable amount of inches
    * @param    inches the distance, in inches, the robot moves
    */
    public void right(float inches){
        int pos = -(int)(inches * Mstep * Math.sqrt(2) / Cw);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setTargetPosition(pos);
        FR.setTargetPosition(-pos);
        BL.setTargetPosition(-pos);
        BR.setTargetPosition(pos);
        FL.setPower(0.3);
        FR.setPower(0.3);
        BL.setPower(0.3);
        BR.setPower(0.3);
        while(FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()){
            if(DEBUGGING)displayStats();
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

       /**
    * turns the robot clockwise a variable amount of degrees
    * @param    degrees the amount of degrees the robot turns
    */
    public void clockwise(int degrees) {
        int pos = (int) (degrees * Cr * Mstep / 360 / Cw);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setTargetPosition(pos);
        FR.setTargetPosition(-pos);
        BL.setTargetPosition(pos);
        BR.setTargetPosition(-pos);
        FL.setPower(0.3);
        FR.setPower(0.3);
        BL.setPower(0.3);
        BR.setPower(0.3);
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
            if(DEBUGGING)displayStats();
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
       /**
    * turns the robot counter clockwise a variable amount of degrees
    * @param    degrees the amount of degrees the robot turns
    */
    public void cClockwise(int degrees){
        int pos = -(int)(degrees * Cr * Mstep / 360 / Cw);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setTargetPosition(-pos);
        FR.setTargetPosition(pos);
        BL.setTargetPosition(-pos);
        BR.setTargetPosition(pos);
        FL.setPower(0.3);
        FR.setPower(0.3);
        BL.setPower(0.3);
        BR.setPower(0.3);
        while(FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()){
            if(DEBUGGING)displayStats();
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    /**
    * tests robot to make sure it is in working order by rotating each wheel forward for a variable amt of time
    * @param    seconds time spent moving motors
    */
    public void testDrive(int seconds) {
        try {
            FL.setPower(0.5);
            sleep(1000);
            FL.setPower(0);

            FR.setPower(0.5);
            sleep(1000);
            FR.setPower(0);

            BL.setPower(0.5);
            sleep(1000);
            BL.setPower(0);

            BR.setPower(0.5);
            sleep(1000);
            BR.setPower(0);
        }catch (Exception e){}
    }
    /**
     * Moves the robot forward for some milliseconds
     * @param milliseconds The time the robot moves
     */
    public void ForwardTime(int milliseconds){
        FL.setPower(0.5);
        FR.setPower(0.5);
        BR.setPower(0.5);
        BL.setPower(0.5);
        try {
            sleep(milliseconds);
        }catch (Exception e){}
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
     }

    /**
     * Moves the robot backward for some milliseconds
     * @param milliseconds The time the robot moves
     */
     public void BackwardTime(int milliseconds){
         FL.setPower(-0.5);
         FR.setPower(-0.5);
         BR.setPower(-0.5);
         BL.setPower(-0.5);
         try {
             sleep(milliseconds);
         }catch (Exception e){}
         FL.setPower(0);
         FR.setPower(0);
         BL.setPower(0);
         BR.setPower(0);
     }

    /**
     * Moves the robot left for some milliseconds
     * @param milliseconds The time the robot moves
     */
     public void LeftTime(int milliseconds){
         FL.setPower(-0.5);
         FR.setPower(0.5);
         BR.setPower(-0.5);
         BL.setPower(0.5);
         try {
             sleep(milliseconds);
         }catch (Exception e){}
         FL.setPower(0);
         FR.setPower(0);
         BL.setPower(0);
         BR.setPower(0);
     }

    /**
     * Moves the robot right for some milliseconds
     * @param milliseconds The time the robot moves
     */
     public void RightTime(int milliseconds){
         FL.setPower(0.5);
         FR.setPower(-0.5);
         BR.setPower(0.5);
         BL.setPower(-0.5);
         try {
             sleep(milliseconds);
         }catch (Exception e){}
         FL.setPower(0);
         FR.setPower(0);
         BL.setPower(0);
         BR.setPower(0);
     }

    /**
     * turns the robot clockwise for some milliseconds
     * @param milliseconds The time the robot turns
     */
     public void ClockwiseTime(int milliseconds)
     {
         FL.setPower(0.5);
         FR.setPower(-0.5);
         BR.setPower(-0.5);
         BL.setPower(0.5);
         try {
             sleep(milliseconds);
         }catch (Exception e){}
         FL.setPower(0);
         FR.setPower(0);
         BL.setPower(0);
         BR.setPower(0);
     }

    /**
     * turns the robot counter clockwise for some milliseconds
     * @param milliseconds The time the robot turns
     */
     public void CounterClockwiseTime(int milliseconds){
         FL.setPower(-0.5);
         FR.setPower(0.5);
         BR.setPower(0.5);
         BL.setPower(-0.5);
         try {
             sleep(milliseconds);
         }catch (Exception e){}
         FL.setPower(0);
         FR.setPower(0);
         BL.setPower(0);
         BR.setPower(0);
     }

     private void displayStats(){
         telemetry.addData("FL encoder","Curr: " + FL.getCurrentPosition() + "  Tar: " + FL.getTargetPosition());
         telemetry.addLine();
         telemetry.addData("FR encoder","Curr: " + FR.getCurrentPosition() + "  Tar: " + FR.getTargetPosition());
         telemetry.addLine();
         telemetry.addData("BL encoder","Curr: " + BL.getCurrentPosition() + "  Tar: " + BL.getTargetPosition());
         telemetry.addLine();
         telemetry.addData("BR encoder","Curr: " + BR.getCurrentPosition() + "  Tar: " + BR.getTargetPosition());
         telemetry.addLine();
         telemetry.addData("FL motor power",FL.getPower());
         telemetry.addLine();
         telemetry.addData("FR motor power",FR.getPower());
         telemetry.addLine();
         telemetry.addData("BL motor power",BL.getPower());
         telemetry.addLine();
         telemetry.addData("BR motor power",BR.getPower());
         telemetry.update();
     }

}
