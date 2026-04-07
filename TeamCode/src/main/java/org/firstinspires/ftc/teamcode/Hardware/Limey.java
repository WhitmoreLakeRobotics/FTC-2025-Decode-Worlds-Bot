package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Field.DecodeField;
import org.firstinspires.ftc.teamcode.Field.DecodeField.TagPose;

public class Limey extends BaseHardware {

    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    private Limelight3A limelight;
    private LLResult result;

    private double tx = 0;
    private double ty = 0;
    private double tagDistance = 0;
    private double tagAngle = 0;
    private int tagID = -1;

    // store tag pose in camera space for AutoAim
    private double tagXCam = 0;
    private double tagYCam = 0;
    private double tagZCam = 0;

    // camera pose in robot space (in meters)  needs updating
    private static final double CAM_X_ROBOT = 0.0;
    private static final double CAM_Y_ROBOT = 0.0;
    private static final double CAM_Z_ROBOT = 13.5 * 0.0254;
    private static final double CAM_YAW_DEG = 0.0;
    private static final double CAM_PITCH_DEG = 0.0;
    private static final double CAM_ROLL_DEG = 0.0;

    private double robotFieldX = Double.NaN;
    private double robotFieldY = Double.NaN;

    private double cameraFieldHeadingDeg = Double.NaN;   // MJD (renamed from robotFieldHeadingDeg)

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        limelight.setPollRateHz(100);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        result = limelight.getLatestResult();

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {

            var tag = result.getFiducialResults().get(0);

            tagID = tag.getFiducialId();
            tx = tag.getTargetXDegrees();
            ty = tag.getTargetYDegrees();

            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose != null) {

                double x = pose.getPosition().x;
                double y = pose.getPosition().y;
                double z = pose.getPosition().z;

                tagDistance = pose.getPosition().z;
                tagAngle = pose.getOrientation().getYaw();

                tagXCam = x;   // MJD
                tagYCam = y;   // MJD
                tagZCam = z;   // MJD

                double fullDistance = Math.sqrt(x*x + y*y + z*z);

                telemetry.addData("Full 3D Distance", "%.2f", fullDistance);
            }

            // Tag pose in robot space (for debug)
            double[] tagRobot = getTagPoseRobotSpace3D();
            if (tagRobot != null) {
                telemetry.addData("TagRobot X", tagRobot[0]);
                telemetry.addData("TagRobot Y", tagRobot[1]);
                telemetry.addData("TagRobot Z", tagRobot[2]);
            }

            // Field map tag pose
            TagPose tagField = DecodeField.getTagPose(tagID);
            if (tagField != null) {
                telemetry.addData("Field Tag X", tagField.x);
                telemetry.addData("Field Tag Y", tagField.y);
                telemetry.addData("Field Tag Heading", tagField.headingDeg);
            }

            // CAMERA FIELD POSE FROM BOTPOSE — NOT ROBOT POSE // MJD
            Pose3D botPose = result.getBotpose();
            if (botPose != null) {

                robotFieldX = botPose.getPosition().x;
                robotFieldY = botPose.getPosition().y;

                cameraFieldHeadingDeg = -botPose.getOrientation().getYaw();   // MJD
                cameraFieldHeadingDeg = ((cameraFieldHeadingDeg % 360) + 360) % 360;   // MJD

                telemetry.addData("BotPose X", robotFieldX);
                telemetry.addData("BotPose Y", robotFieldY);
                telemetry.addData("Camera Heading", cameraFieldHeadingDeg);   // MJD
            }

            telemetry.addData("Limelight", "VALID TARGET");
            telemetry.addData("Tag ID", tagID);
            telemetry.addData("tx", "%.2f°", tx);
            telemetry.addData("ty", "%.2f°", ty);
            telemetry.addData("Distance", "%.2f", tagDistance);
            telemetry.addData("Yaw", "%.2f°", tagAngle);
            telemetry.addData("Latency", "%.1f ms", result.getTargetingLatency());

        } else {
            tagID = -1;

            telemetry.addData("Limelight", "NO TARGET");
        }
    }

    @Override
    void stop() {}

    public void method(){}

    // --- CAMERA-SPACE TAG POSE (X,Y,Z) --- // MJD
    public double[] getTagPoseCameraSpace3D() {   // MJD
        if (tagID == -1) return null;            // MJD
        return new double[]{ tagXCam, tagYCam, tagZCam };   // MJD
    }

    // --- CAMERA-SPACE TAG YAW (degrees) --- // MJD
    public double getTagYawCameraSpaceDeg() {     // MJD
        if (tagID == -1) return Double.NaN;      // MJD
        return tagAngle;                         // MJD
    }


    // Returns: [x, y, z, roll, pitch, yaw]
    public double[] getBotPose() {
        if (result == null) return null;

        Pose3D bot = result.getBotpose();
        if (bot == null) return null;

        return new double[]{
                bot.getPosition().x,
                bot.getPosition().y,
                bot.getPosition().z,
                bot.getOrientation().getRoll(),
                bot.getOrientation().getPitch(),
                bot.getOrientation().getYaw()
        };
    }

    // 2D tag pose in ROBOT SPACE (yaw-only)
    public double[] getTagPoseRobotSpace() {
        if (result == null) return null;
        if (!result.isValid()) return null;
        if (result.getFiducialResults().isEmpty()) return null;

        Pose3D pose = result.getFiducialResults().get(0).getTargetPoseCameraSpace();
        if (pose == null) return null;

        double camX = pose.getPosition().x;
        double camZ = pose.getPosition().z;

        double robotX = camZ;      // forward  // MJD FIX
        double robotY = -camX;     // left     // MJD FIX

        return new double[]{
                robotX,
                robotY,
                pose.getPosition().z,
                pose.getOrientation().getRoll(),
                pose.getOrientation().getPitch(),
                pose.getOrientation().getYaw()
        };
    }

    // 3D tag pose in ROBOT SPACE (for full 3D aiming)
    public double[] getTagPoseRobotSpace3D() {
        if (result == null) return null;
        if (!result.isValid()) return null;
        if (result.getFiducialResults().isEmpty()) return null;

        Pose3D pose = result.getFiducialResults().get(0).getTargetPoseCameraSpace();
        if (pose == null) return null;

        double cx = pose.getPosition().x;
        double cy = pose.getPosition().y;
        double cz = pose.getPosition().z;

        double rx = cz;      // forward   // MJD FIX
        double ry = -cx;     // left      // MJD FIX
        double rz = -cy;     // up        // MJD FIX

        double tagRobotX = CAM_X_ROBOT + rx;
        double tagRobotY = CAM_Y_ROBOT + ry;
        double tagRobotZ = CAM_Z_ROBOT + rz;

        return new double[]{
                tagRobotX,
                tagRobotY,
                tagRobotZ
        };
    }

    // Returns CAMERA pose in field space — NOT robot pose // MJD
    public double[] getRobotPoseFieldSpace() {
        return new double[]{ robotFieldX, robotFieldY, cameraFieldHeadingDeg };   // MJD
    }

    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public int getTagID() { return tagID; }
    public double getTagDistance() { return tagDistance; }

    public double getTagAngle() {
        if(result == null) return Double.NaN;
        if(!result.isValid()) return Double.NaN;
        if(tagID == -1) return Double.NaN;
        return tagAngle;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setHardwareMap(HardwareMap hw) {
        this.hardwareMap = hw;
    }
}
