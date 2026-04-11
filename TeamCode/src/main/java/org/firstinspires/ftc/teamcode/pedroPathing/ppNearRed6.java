package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants.pathConstraints;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

//Verity + Rory
@Autonomous(name = "ppNearRed6", group = "PP")
public class ppNearRed6 extends OpMode {

    Robot robot = new Robot();


    private String thisUpdate = "0";
    private TelemetryManager telemetryMU;
    private stage currentStage = stage._00_unknown;
    private ElapsedTime runtime = new ElapsedTime();
    public boolean End = false;



    public static Follower follower;
    public Pose currentPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(follower.getPose().getHeading()));

    public static Pose startPose = new Pose(110, 135, Math.toRadians(0)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(96, 96, Math.toRadians(50)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAP = new Pose(20, 20, Math.toRadians(10));
    public static Pose pickup1aPose = new Pose(120, 80, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup1bPose = new Pose(120, 60, Math.toRadians(190)); // (First Set) of Artifacts picked up.
    public static Pose pickupgatePose = new Pose(140, 50, Math.toRadians(200));
    public static Pose pickup1cPose = new Pose(120, 40, Math.toRadians(180));
    public static Pose spikeR2start = new Pose (110,60,Math.toRadians(-90));
    public static Pose spikeR2end = new Pose (128,63,Math.toRadians(-90)); // was 130 60
    public static Pose spikeR1start = new Pose (110,84,Math.toRadians(-90));
    public static Pose spikeR1end = new Pose (128,77,Math.toRadians(-90)); // was 130 84
    //public static Pose spikeR3start = new Pose (110,36,Math.toRadians(-90));
   // public static Pose spikeR3end = new Pose (130,36,Math.toRadians(-90));
   // public static Pose Rcorner = new Pose(11.3,8.9,Math.toRadians(190));


    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup1a, grabPickup1b, grabPickup1c, scorePickup1, grabPickup2a, grabPickup2b, scorePickup2, goEndPose, goEndPose2, endPath;
    private PathChain cyclePickup1, interruptedPickup, spikeR2, spikeR1;


    public void buildPaths() {
        cyclePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1aPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1aPose.getHeading())

                .addPath(new BezierLine(pickup1aPose, pickup1bPose))
                .setLinearHeadingInterpolation(pickup1aPose.getHeading(), pickup1bPose.getHeading())

                .addPath(new BezierCurve(pickup1bPose, scorePoseAP))
                .setLinearHeadingInterpolation(pickup1bPose.getHeading(), scorePose.getHeading())

                .build();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build();

        spikeR2 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeR2start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeR2start.getHeading())

                .addPath (new BezierLine(spikeR2start,spikeR2end))
                .setLinearHeadingInterpolation(spikeR2start.getHeading(), spikeR2end.getHeading())
                .build();

        spikeR1 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeR1start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeR1start.getHeading())

                .addPath (new BezierLine(spikeR1start,spikeR1end))
                .setLinearHeadingInterpolation(spikeR1start.getHeading(), spikeR1end.getHeading())
                .build();



    }

    @Override
    public void init() {

// NAJ CompBotConstants is a file/class that contains the definition of the gryo and drive motors among other things
        //super.init();
        follower = CompBotConstants.createFollower(hardwareMap);
        buildPaths();
        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.setStartingPose(startPose);
        follower.update();
//  pedroPanelsTelemetry.init();
        Drawing.init();
        telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();

// disp[lay starting postition
        telemetryMU.addData("initialized postition - Update ", thisUpdate);
// Feedback to Driver Hub for debugging
        updateTelemetry();

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
    }


    @Override
    public void init_loop() {
        //super.init_loop();


        robot.init_loop();

    }

    @Override
    public void start () {
        //super.start();
        robot.start();
    }

    @Override
    public void loop() {

        updateTelemetry();
        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.autonLoop();
        follower.update();
        switch (currentStage) {
            case _00_unknown:
                currentStage = stage._10_preStart;
                break;

            case _10_preStart:
                currentStage = stage._20_Prelaunch;
                break;

            case _20_Prelaunch:
                    if (!follower.isBusy()) {
                        follower.followPath(scorePreload, true);
                        robot.autoRPM.Measure = true;


                        // follower.update();

                    currentStage = stage._25_ScorePrelaunch;
                }

                break;
            case _25_ScorePrelaunch:
                if (!follower.isBusy()){
                    dolaunch_process();
                    currentStage = stage._30_Spike1;
                }

                 break;
            case _30_Spike1:
                if (robot.sensors.NoBalls || runtime.milliseconds() >= 1000) {
                    endlaunch_process();
                    follower.followPath(spikeR2);
                    telemetryMU.addData("Cornor pickup", follower.getPose());
                    currentStage = stage._35_DrivetoLaunch1a;
                }
                break;

            case _35_DrivetoLaunch1a:
                if(follower.isBusy()){
                    AreYouSure(stage._40_Launch1);
                }
                break;

            case _40_Launch1:
                if(!follower.isBusy()){
                  dolaunch_process();
                  currentStage = stage._50_Spike2;
                }
                break;

            case _50_Spike2:
                if(robot.sensors.NoBalls || runtime.milliseconds() >= 1000){
                    endlaunch_process();
                    follower.followPath(spikeR1); //change to new
                    currentStage = stage._60_DrivetoLaunch1b;
                }
                break;

            case _60_DrivetoLaunch1b:
                if(follower.isBusy()){
                    AreYouSure(stage._65_Launch2);
                }
                break;

            case _65_Launch2:
                if(!follower.isBusy()){
                    dolaunch_process();
                    currentStage = stage._1000_end;
                }
                break;

            case _1000_end:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    stop();
                    runtime.reset();
                    End = true;
                }
                break;



        }

    }



    @Override
    public void stop () {
        //super.stop();
        robot.stop();
    }
    private enum stage {

        _00_unknown,
        _10_preStart,
        _20_Prelaunch,
        _25_ScorePrelaunch,
        _30_Spike1,
        _35_DrivetoLaunch1a,
        _40_Launch1,
        _50_Spike2,
        _60_DrivetoLaunch1b,
        _65_Launch2,
        _70_Spike3,
        _80_DrivetoLaunch1c,
        _90_Launch3,
        _100_Gatepos1,
        _110_PickupGate1,
        _120_DrivetoLaunch2a,
        _125_Launch4,
        _130_Tunnelpickup,
        _160_Launch5,
        _170_,
        _1000_end;


    }

    private void dolaunch_process(){

        robot.launcherBlocker.cmdUnBlock();
        robot.transitionRoller.cmdSpin();
        robot.intake.cmdFoward();
        runtime.reset();

    }

    private void endlaunch_process(){

        robot.launcherBlocker.cmdBlock();
        robot.autoRPM.Measure = false;
        robot.launcher.cmdStop();

    }

    private void AreYouSure(stage NextStage) {

        //telemetryMU.addData("pathPose2", pickup1bPose);
        //telemetryMU.addData("scorePose", scorePoseAP);
        if (follower.isBusy()) { //we are still running path
//telemetryMU.addData("check intake status", robot.intake.AtIntakeStop); intake.AtIntakeStop is never set to false
            if (robot.sensors.allFilled) {
                telemetryMU.addLine("Intake stopped - break follower");
                // we've got 3 artifacts, stop the path and return to scorePose
                follower.breakFollowing();
                newPath();
                //   robot.autoRPM.Measure = true; // start fly wheels
                robot.autoRPM.Measure = true;
                currentStage = NextStage;
                runtime.reset();

            } else if (follower.getCurrentTValue() > 0.75) { //the path is almost done
                //  robot.autoRPM.Measure = true; //start fly wheels
                robot.autoRPM.Measure = true;
            }
        } else {// path is complete we are back at scorePose move to launch
            robot.autoRPM.Measure = true;
            currentStage = NextStage;
        }
    }


    private void updateTelemetry () {
        telemetryMU.addData("Follower Busy?", follower.isBusy());
        telemetryMU.addData("Current Stage", currentStage);
        telemetryMU.addData("x", follower.getPose().getX());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        //  telemetryMU.addData("LAST Pose", lastPose);
        //  telemetryMU.addData("Current Target Pose", currentTargetPose);
        telemetryMU.addData("breakingStrength", pathConstraints.getBrakingStrength());
        telemetryMU.addData("breakstart ", pathConstraints.getBrakingStart());
        telemetryMU.addData("drivepid P", follower.constants.coefficientsDrivePIDF.P);
        telemetryMU.addData("drivepid D", follower.constants.coefficientsDrivePIDF.D);
        telemetryMU.addData("drivepid F", follower.constants.coefficientsDrivePIDF.F);
        telemetryMU.addData("CONSTRAINTS", "");
        telemetryMU.addData("Tvalue (% complete)", follower.pathConstraints.getTValueConstraint());
        telemetryMU.addData("Current tValue", follower.getCurrentTValue());
        telemetryMU.addData("Velocity Constraint", follower.pathConstraints.getVelocityConstraint());
        telemetryMU.addData("Current Velocity", follower.getVelocity());
        telemetryMU.addData("Trans constraint", follower.pathConstraints.getTranslationalConstraint());
        // telemetryMU.addData("current Trans", follower.getTranslationalError());
        telemetryMU.addData("Heading Constraint", follower.pathConstraints.getHeadingConstraint());

        telemetryMU.update();
        Drawing.drawDebug(follower);
    }

    private void newPath(){
        interruptedPickup = follower.pathBuilder()
                .addPath (new BezierLine(currentPose, scorePose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scorePose.getHeading())
                .build();
        follower.followPath(interruptedPickup,true);

    }


}
