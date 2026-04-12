package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftOff {

    public HardwareMap hardwareMap = null;// will be set in Child class

    public Mode CurrentMode;
    //public Servo Thruster;
    public boolean UP = false;

    public static final double LiftOff = 1.0;
    public static final double Stop = 0.0;

    public Telemetry telemetry = null;


    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init() {



        //Thruster = hardwareMap.get(Servo.class, "Thruster");

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop() {

    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start() {

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop() {

    }

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    void stop() {

    }

    public void cmdUp(){
        CurrentMode = Mode.UP;
        //Thruster.setPosition(LiftOff);
        UP = true;
    }





    public void cmdStop(){
        CurrentMode = Mode.STOP;
        //Thruster.setPosition(Stop);
        UP = false;
    }



    public enum Mode{
        UP,
        STOP
    }

}


