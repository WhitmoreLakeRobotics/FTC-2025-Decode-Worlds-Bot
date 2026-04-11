package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;


public class TrapezoidAutoAim extends BaseHardware{ // naj added this to comply with standard baseHardware class

    private Limey limey;
    // private Turret turret;
    private DriveTrain driveTrain;
   // private Robot robot;

    public TurretColor CurrentTurretColor;
    public Mode CurrentMode;

    public Telemetry telemetry = null;
    public HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();
    public boolean PrimitiveDriver = false;
    public double YawDif = 0;
    public boolean JackHappy = false;
    public static double heading = 0;

    private  Robot robot;

    /*
    public TrapezoidAutoAim(Limey limey,DriveTrain driveTrain, Telemetry telemetry,HardwareMap hardwareMap){
        this.limey = limey;
        this.driveTrain = driveTrain;
        this.telemetry = telemetry;
        this.hardwareMap =hardwareMap;
        this.CurrentMode = Mode.NotTrying;
        //this.CurrentTurretColor = TurretColor.Unknown;
    }

     */

    public void init(){

        //this.limey = limey;
        //this.driveTrain = driveTrain;
        //this.telemetry = telemetry;
        // this.hardwareMap =hardwareMap;
        this.CurrentMode = Mode.NotTrying;


    }

    public void init_loop(){


    }

    public void start(){

    }

    public void loop(){
        //runtime.log("Position");
        //limey.getTx();

        if(robot.limey == null) return;
        if(driveTrain == null) return;

        if(robot.limey.getTagID() > -1) {
            YawDif = robot.limey.getTagAngle() * 0.250;
        }

        if(!PrimitiveDriver && robot.turret.ableToAim) {
            if (CurrentTurretColor == TurretColor.Red) {
                if (robot.limey.getTagID() == 24) {
                    if (robot.limey.getTx() > 72 + YawDif) {
                        JackHappy = false;
                        CurrentMode = Mode.Targeting;
                        //maybe change to ty
                        if(robot.limey.getTx() <= 36 || robot.limey.getTx() >= 108){
                          //  driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 4),0.35);
                            heading = heading + 3;
                            robot.turret.cmdRightFar();
                        }else{
                           // driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 1), 0.35);
                            heading = heading + 1;
                            robot.turret.cmdRight();
                        }
                    } else if (robot.limey.getTx() < 72 + YawDif) {
                        JackHappy = false;
                        CurrentMode = Mode.Targeting;
                        if(robot.limey.getTx() <= 36 || robot.limey.getTx() >= 108){
                           // driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 4),0.35);
                            heading = heading - 3;
                            robot.turret.cmdLeftFar();
                        }else{
                           // driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 1), 0.35);
                            heading = heading - 1;
                            robot.turret.cmdLeft();
                        }
                    } else {
                        robot.turret.cmdNo();
                        if(robot.limey.getTx() == 72 + YawDif) {
                            JackHappy = true;
                            CurrentMode = Mode.Target_Acquired;
                        }

                    }
                } else {
                    robot.turret.cmdNo();
                    CurrentMode = Mode.Target_NotFound;
                }
            }
            if (CurrentTurretColor == TurretColor.Blue) {
                if (robot.limey.getTagID() == 20) {
                    if (robot.limey.getTx() > 72 + YawDif) {
                        JackHappy = false;
                        CurrentMode = Mode.Targeting;
                       // robot.turret.cmdRight();
                        if(robot.limey.getTx() <= 36 || robot.limey.getTx() >= 108){
                          //  driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 4),0.35);
                            heading = heading + 3;
                            robot.turret.cmdRightFar();
                        }else{
                           // driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 1), 0.35);
                            heading = heading + 1;
                            robot.turret.cmdRight();
                        }
                    } else if (robot.limey.getTx() < 72 + YawDif) {
                        JackHappy = false;
                        CurrentMode = Mode.Targeting;
                        //robot.turret.cmdLeft();
                        if(robot.limey.getTx() <= 36 || robot.limey.getTx() >= 108){
                            //driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 4),0.35);
                            heading = heading - 3;
                            robot.turret.cmdLeftFar();
                        }else{
                            //driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 1), 0.35);
                            heading = heading - 1;
                            robot.turret.cmdLeft();
                        }
                    } else {
                        robot.turret.cmdNo();
                        if(robot.limey.getTx() == 72 + YawDif){
                            JackHappy = true;
                            CurrentMode = Mode.Target_Acquired;
                        }

                    }
                } else {
                    robot.turret.cmdNo();
                    CurrentMode = Mode.Target_NotFound;
                }
            }
        }

        // if(CurrentTurretColor == TurretColor.NoAuto || CurrentTurretColor == TurretColor.Unknown){
        //    PrimitiveDriver = true;
        // }


        if(CurrentMode == Mode.Targeting && robot.limey.getTagID() == -1){
            CurrentMode = Mode.Target_NotFound;
        }

       // if(heading > 170){
            //driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 340),0.70);
            //heading = heading - 335;
      //  } else if (heading < -170){
            //driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 340),0.70);
           // heading = heading + 335;
      //  }else {

      //  }











    }

    public void stop(){

    }

    public void followMe1(){
        if(robot.limey.getTagID() == 1){
            if (robot.limey.getTx() > 72){
                driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 1),0.35);
            }else if(robot.limey.getTx() < 72){
                driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 1),0.35);
            }else{

            }

            if(robot.limey.getTagDistance() > 3 && robot.limey.getTagDistance() < 5){
                driveTrain.cmdDriveBySensors(6,driveTrain.getCurrentHeading(),0.35,driveTrain.getCurrentHeading());
            }else if(robot.limey.getTagDistance() > 5 && robot.limey.getTagDistance() > 3){
                driveTrain.cmdDriveBySensors(12,driveTrain.getCurrentHeading(),0.60,driveTrain.getCurrentHeading());
            }else{

            }
        }
    }



    public enum Mode{
        Targeting,
        Target_Acquired,
        Target_NotFound,
        NotTrying

    }

    public enum TurretColor{
        Red,
        Blue,
        NoAuto,
        Unknown
    }

}



