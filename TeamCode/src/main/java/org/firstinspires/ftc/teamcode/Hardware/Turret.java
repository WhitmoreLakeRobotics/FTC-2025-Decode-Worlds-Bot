package org.firstinspires.ftc.teamcode.Hardware;

import static java.lang.Math.clamp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;


public class Turret extends BaseHardware {

    public Telemetry telemetry = null;

    private Servo rightyTighty;
    private Servo leftyLoosy;

    // tunable pid constants
    private double kP = 0.012;
    private double kI = 0.000;
    private double kD = 0.0012;

    // angle state
    private double targetAngle = 0;
    private double currentAngle = 0;
    private double lastError = 0;
    private double integral = 0;

    private double currentPosition = 0.0;
    public boolean ableToAim = true;

    // limits
    private final double MIN_DEG = -170;
    private final double MAX_DEG = 170;

    // servo limits
    private final double MIN_POS = 0.05;
    private final double MAX_POS = 0.95;

    // saftey: max angle change per loop (deg)
    private final double MAX_VELOCITY = 4.0;
    private ElapsedTime runtime = new ElapsedTime();

    public TrapezoidAutoAim trapezoidAutoAim;
    public HardwareMap hardwareMap = null;

    private boolean driverOverride = false;              // MJD
    private ElapsedTime overrideTimer = new ElapsedTime(); // MJD

    private double softMinDeg = MIN_DEG;                 // MJD
    private double softMaxDeg = MAX_DEG;                 // MJD

    // Hard limit helpers — MJD
    public boolean isAtLeftLimit() { return currentAngle <= MIN_DEG + 0.5; }   // MJD
    public boolean isAtRightLimit() { return currentAngle >= MAX_DEG - 0.5; }  // MJD

    private double clampToHardLimits(double angle) {
        return clamp(angle, MIN_DEG, MAX_DEG);
    }
    // Soft limit helpers — MJD
    public void updateSoftLimits(double robotHeadingDeg) {
        //Soft limit logic — tune later
        if (robotHeadingDeg > 150 && robotHeadingDeg < 210) {
            softMinDeg = -120;
            softMaxDeg = 120;
        } else {
            softMinDeg = MIN_DEG;
            softMaxDeg = MAX_DEG;
        }
    }
    private double clampToSoftLimits(double angle) {
        return clamp(angle, softMinDeg, softMaxDeg);
    }
    // Safe angle validation — MJD
    public boolean isSafeAngle(double angle) {
        if (angle < softMinDeg || angle > softMaxDeg) return false;
        if (angle < MIN_DEG || angle > MAX_DEG) return false;
        return true;
    }

    @Override
    public void init() {
        currentAngle = 0;
        targetAngle = 0;

        leftyLoosy = hardwareMap.get(Servo.class,"leftyLoosy");
        rightyTighty = hardwareMap.get(Servo.class,"rightyTighty");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        // DRIVER OVERRIDE LOGIC — MJD

        double stick = 0; // Replace with joystick input when integrated — MJD

        if (Math.abs(stick) > 0.6) {     // driver touching stick
            driverOverride = true;
            overrideTimer.reset();
        } else if (driverOverride && overrideTimer.seconds() > 0.5) {
            driverOverride = false;
        }
        // --- PID CALCULATION ---
        if (!driverOverride) {   //only run PID when NOT overridden

            double error = targetAngle - currentAngle;

            // anti-windup
            if (Math.abs(error) < 25) {
                integral += error;
            } else {
                integral = 0;
            }

            double derivative = error - lastError;
            lastError = error;

            double output = (kP * error) + (kI * integral) + (kD * derivative);

            // velocity limit
            output = clamp(output, -MAX_VELOCITY, MAX_VELOCITY);

            // update angle
            currentAngle += output;
            currentAngle = clamp(currentAngle, MIN_DEG, MAX_DEG);

            // map to servo position
            double pos = (currentAngle - MIN_DEG) / (MAX_DEG - MIN_DEG);
            pos = clamp(pos, MIN_POS, MAX_POS);

            //leftyLoosy.setPosition(pos);
            //rightyTighty.setPosition(pos);

            telemetry.addData("Turret Target", targetAngle);
            telemetry.addData("Turret Angle", currentAngle);
            telemetry.addData("Turret Pos", pos);
            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.addData("Driver Override", driverOverride); // MJD
            telemetry.addData("SoftMin", softMinDeg);             // MJD
            telemetry.addData("SoftMax", softMaxDeg);             // MJD
        }
    }

    @Override
    void stop() {

    }

    public void cmdLeft() {
        rightyTighty.setPosition(0.25);
        leftyLoosy.setPosition(0.25);
        currentPosition = currentPosition + 1;
        trapezoidAutoAim.runtime.reset();
    }
    public void cmdRight() {
        rightyTighty.setPosition(-0.25);
        leftyLoosy.setPosition(-0.25);
        currentPosition = currentPosition - 1;
        trapezoidAutoAim.runtime.reset();
    }

    public void cmdLeftFar() {
        rightyTighty.setPosition(1);
        leftyLoosy.setPosition(1);
        currentPosition = currentPosition + 4;
        trapezoidAutoAim.runtime.reset();
    }
    public void cmdRightFar() {
        rightyTighty.setPosition(-1);
        leftyLoosy.setPosition(-1);
        currentPosition = currentPosition - 4;
        trapezoidAutoAim.runtime.reset();
    }

    public void cmdTurn(int targetPosition, double turretSpeed){

        if(targetPosition > currentPosition){
            currentPosition = currentPosition - (turretSpeed * 4.0);
            rightyTighty.setPosition(turretSpeed);
            leftyLoosy.setPosition(turretSpeed);
           // cmdTurn(targetPosition, turretSpeed); // turret speed - 0.25 so slows down as gets closer to target
        }else if(targetPosition < currentPosition){
            currentPosition = currentPosition + (turretSpeed * 4.0);
            rightyTighty.setPosition(turretSpeed);
            leftyLoosy.setPosition(turretSpeed);
           // cmdTurn(targetPosition, turretSpeed);
        }else{
            cmdNo();
        }

        if(!CommonLogic.inRange(currentPosition, targetPosition, 1)){
            if(CommonLogic.inRange(currentPosition, targetPosition, 10)){
                cmdTurn(targetPosition, turretSpeed - (turretSpeed * 0.1));
                ableToAim = false;
            }else{
                cmdTurn(targetPosition, turretSpeed);
                ableToAim = false;
            }
        }else{
            cmdNo();
            ableToAim = true;
        }
    }





    public void cmdNo() {
        rightyTighty.setPosition(0);
        leftyLoosy.setPosition(0);
        trapezoidAutoAim.runtime.reset();
    }
    public void manualControl(double stick) {
        double speedDegPerLoop = 3.0;
        double newAngle = currentAngle + (stick * speedDegPerLoop);

        newAngle = clamp(newAngle, MIN_DEG, MAX_DEG);

        setTargetAngle(newAngle);
    }
    public void setTargetAngle(double angle) {   // MJD
        angle = clampToHardLimits(angle);        // MJD
        angle = clampToSoftLimits(angle);        // MJD

        if (isSafeAngle(angle)) {                // MJD
            targetAngle = angle;
        }
    }
    public double getCurrentAngle() {
        return currentAngle;
    }
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
