package org.firstinspires.ftc.teamcode.Hardware;

import static java.lang.Math.clamp;

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
    private int milliDeg = 50;

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

    public Limey limey = null;              // MJD

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
    public void init_loop() { }

    @Override
    public void start() { }

    @Override
    public void loop() {

        // --- MJD: derive currentAngle from servo position ---
        double posRead = leftyLoosy.getPosition();   // assume left is primary
        currentAngle = MIN_DEG + posRead * (MAX_DEG - MIN_DEG);
        currentAngle = clamp(currentAngle, MIN_DEG, MAX_DEG);

        // --- DRIVER OVERRIDE LOGIC — external control via setDriverOverride/manualControl ---

        if (!driverOverride) {
            // --- AUTO-AIM USING LIMELIGHT / LIMEY — MJD ---
            if (limey != null && limey.getTagID() != -1) {
                // Use tx (horizontal offset in degrees) to adjust turret
                double tx = limey.getTx();   // positive = target to the right
                double desired = currentAngle + tx;   // simple relative aim

                desired = clampToHardLimits(desired);
                desired = clampToSoftLimits(desired);

                if (isSafeAngle(desired)) {
                    targetAngle = desired;
                }
            }
            // else: no tag → hold last targetAngle
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

            // MJD — dual-servo mapping: one reversed
            leftyLoosy.setPosition(pos);
            rightyTighty.setPosition(1.0 - pos);

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
    void stop() { }

    /*
    // OLD incremental commands — these bypass PID and are not safe with new control
    public void cmdLeft() {
        rightyTighty.setPosition(rightyTighty.getPosition() + 0.002778); //maybe 0.000926
        leftyLoosy.setPosition(leftyLoosy.getPosition() - 0.002778);
        currentPosition = currentPosition - 1;
        trapezoidAutoAim.runtime.reset();
    }
    public void cmdRight() {
        rightyTighty.setPosition(rightyTighty.getPosition() - 0.002778);
        leftyLoosy.setPosition(leftyLoosy.getPosition() + 0.002778);
        currentPosition = currentPosition + 1;
        trapezoidAutoAim.runtime.reset();
    }

    public void cmdLeftFar() {
        rightyTighty.setPosition(rightyTighty.getPosition() + 0.008334); //maybe 0.002778
        leftyLoosy.setPosition(leftyLoosy.getPosition() - 0.008334);
        currentPosition = currentPosition - 3;
        trapezoidAutoAim.runtime.reset();
    }
    public void cmdRightFar() {
        rightyTighty.setPosition(rightyTighty.getPosition() - 0.008334);
        leftyLoosy.setPosition(leftyLoosy.getPosition() + 0.008334);
        currentPosition = currentPosition + 3;
        trapezoidAutoAim.runtime.reset();
    }


    public void cmdTurn(int targetPosition, double turretSpeed){

        if(targetPosition > currentPosition){
        driverOverride = true;
            currentPosition = currentPosition - (turretSpeed * 4.0);
            rightyTighty.setPosition(turretSpeed);
            leftyLoosy.setPosition(-turretSpeed);
        }else if(targetPosition < currentPosition){
         driverOverride = true;
            currentPosition = currentPosition + (turretSpeed * 4.0);
            rightyTighty.setPosition(-turretSpeed);
            leftyLoosy.setPosition(turretSpeed);
        }else{
            cmdNo();
        }

        if(!CommonLogic.inRange(currentPosition, targetPosition, 1)){ //can be 0.5
            if(CommonLogic.inRange(currentPosition, targetPosition, 10)){
                ableToAim = false;
                cmdTurn(targetPosition, turretSpeed - (turretSpeed * 0.1));
            }else{
                ableToAim = false;
                cmdTurn(targetPosition, turretSpeed);
            }
        }else{
         driverOverride = false;
            cmdNo();
            ableToAim = true;
        }
    }

    public void cmdNo() {
        rightyTighty.setPosition(0);
        leftyLoosy.setPosition(0);
        trapezoidAutoAim.runtime.reset();
    }
    */

    // --- DRIVER OVERRIDE API — call from TeleOp --- // MJD
    public void manualControl(double stick) {   // MJD
        double speedDegPerLoop = 3.0;
        double newAngle = currentAngle + (stick * speedDegPerLoop);

        newAngle = clampToHardLimits(newAngle);
        newAngle = clampToSoftLimits(newAngle);

        if (isSafeAngle(newAngle)) {
            targetAngle = newAngle;
        }
    }

    public void setDriverOverride(boolean override) {   // MJD
        if (override && !driverOverride) {
            overrideTimer.reset();
        }
        driverOverride = override;
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
