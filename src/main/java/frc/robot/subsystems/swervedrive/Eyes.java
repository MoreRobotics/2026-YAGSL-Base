/*
 * This subsytem controls robot vision. It uses the LimelightHelpers class
 * and contains methods that are used to track april tags and gather
 * positional data from them. This data is then used in the swerve
 * subsystem to update the robot's odometry using the pose estimator.
 * 
 * parameters:
 * none
 */


 package frc.robot.subsystems.swervedrive;


 import frc.robot.Constants;
import java.util.List;
 
 import com.pathplanner.lib.path.GoalEndState;
 import com.pathplanner.lib.path.IdealStartingState;
 import com.pathplanner.lib.path.PathConstraints;
 import com.pathplanner.lib.path.PathPlannerPath;
 import com.pathplanner.lib.path.Waypoint;
 
 import edu.wpi.first.wpilibj.DriverStation;
 import edu.wpi.first.wpilibj.DriverStation.Alliance;
 import edu.wpi.first.wpilibj.Timer;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Pose3d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Rotation3d;
 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.math.util.Units;
 import edu.wpi.first.networktables.NetworkTable;
 import edu.wpi.first.networktables.NetworkTableEntry;
 import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
 import frc.robot.subsystems.swervedrive.*;
import limelight.Limelight;
 
 
 
 public class Eyes extends SubsystemBase {
 
     // Swerve subsystem for pose estimator
     SwerveSubsystem s_Swerve;
     LimelightHelpers r_LimelightHelpers;

     public StructPublisher<Pose2d> targetHub;
     public StructPublisher<Pose2d> estimatedRobotPosePublisher;
 
     // create objects and variables
     public LimelightHelpers limelight;
     public double tx;
     public double ty;
     public double ta;
     public double tID;
     public double txnc;
     public double tync;
 
     
 
 
    
     public boolean controllerRumble = false;
     public PathPlannerPath reefPath;
     public boolean closeToReef = false;
     public double closestDistance;
     public boolean doRejectUpdate;
     // constuctor
     public Eyes(SwerveSubsystem swerve) {
 
         s_Swerve = swerve;
         // reefPath = closestReefpath(-1);


    estimatedRobotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("/EstimatedRobotPose", Pose2d.struct).publish(PubSubOption.sendAll(true));//publisher needs arguments
    

    targetHub = NetworkTableInstance.getDefault().getStructTopic("/TargetHubPose", Pose2d.struct).publish(PubSubOption.sendAll(true));
         
     }
 
  
     /*
      * This method will gather all of the positional data of the limelight target.
      * 
      * parameters:
      * none
      * 
      * returns;
      * none
      */
     public void updateData() {
 
         /* 
          * get data from limelight target
          * tx = x position in degrees in limelight FOV
          * ty = y position in degrees in limelight FOV
          * ta = pitch in degrees in limelight FOV
          * tID = target ID number
          */
         NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-left");
         NetworkTableEntry tx = table.getEntry("tx");
         NetworkTableEntry ty = table.getEntry("ty");
         NetworkTableEntry ta = table.getEntry("ta");

         NetworkTable left = NetworkTableInstance.getDefault().getTable("limelight-right");
         NetworkTableEntry txLeft = left.getEntry("tx");
         NetworkTableEntry tyLeft = left.getEntry("ty");
         NetworkTableEntry taLeft = left.getEntry("ta");
 
 
         //read values periodically
         double x = tx.getDouble(tID);
         double y = ty.getDouble(0.0);
         double area = ta.getDouble(0.0);

         double xLeft = txLeft.getDouble(tID);
         double yLeft = tyLeft.getDouble(0.0);
         double areaLeft = taLeft.getDouble(0.0);
 
         //post to smart dashboard periodically
         SmartDashboard.putNumber("LimelightX", x);
         SmartDashboard.putNumber("LimelightY", y);
         SmartDashboard.putNumber("LimelightArea", area);

         SmartDashboard.putNumber("Limelight-LeftX", xLeft);
         SmartDashboard.putNumber("Limelight-LeftY", yLeft);
         SmartDashboard.putNumber("LimelightArea-Left", areaLeft);
 
         // tx = LimelightHelpers.getTX("limelight");
         // ty = LimelightHelpers.getTY("limelight");
         // ta = LimelightHelpers.getTA("limelight");
         //   tID = LimelightHelpers.getFiducialID("limelight");
 
         // txnc = LimelightHelpers.getTXNC("limelight");  // Horizontal offset from principal pixel/point to target in degrees
         // tync = LimelightHelpers.getTYNC("limelight");  // Vertical  offset from principal pixel/point to target in degrees
 
         LimelightHelpers.setPipelineIndex("limelight-right", 0);
         LimelightHelpers.setPipelineIndex("limelight-left", 0);
 
         // log target data
         SmartDashboard.putNumber("AprilTagID", tID);
         


 
     }
 
     /*
      * This method will wrap all target data into an array for easy access.
      * 
      * Array indexes:
      * [0] = x
      * [1] = y
      * [2] = a (pitch)
      * [3] = ID
      */
     public double[] getDataPackage() {
 
         double[] data = {
             tx,
             ty,
             ta,
             tID
         };
 
         return data;
     }
 
     /*
      * This method will return the pose of the robot based upon the pose of a detected apriltag
      * 
      * parameters:
      * none
      * 
      * returns:
      * robot pose      (Pose2d)
      */
     public Pose2d getRobotPose() {
 
        // Pose2d pose;

 
         
         Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-right");
         return pose;
         
         
     }

     public Pose2d getRobotPoseLeft() {
        Pose2d pose;
 
         
         pose = LimelightHelpers.getBotPose2d_wpiBlue("limelight-left");
         return pose;
     }

     public double getTargetRotation() {
        Pose2d robotPose = s_Swerve.m_PoseEstimator.getEstimatedPosition();

        Pose2d targetPose = getTargetPose();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double targetX = targetPose.getX();
        double targetY = targetPose.getY();

        double angle =  (Math.atan((targetY - robotY) / (targetX - robotX)) * (180 / Math.PI));
        SmartDashboard.putNumber("RobotX", robotX);
        SmartDashboard.putNumber("RobotY", robotY);
        SmartDashboard.putNumber("TargetX", targetX);
        SmartDashboard.putNumber("TagretY", targetY);



        // if (robotX > targetX) {

        //     angle = angle + 180;

        // }

         
        // SmartDashboard.putNumber(" inverted angle", -angle);
         if (s_Swerve.isRedAlliance()) {
             if(s_Swerve.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees() > 0)
             {
                 return angle+(180 - s_Swerve.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
             }
             else {
                 return angle + (-180 - s_Swerve.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
             }
         }
         else{
            return angle;
         }

     }

     public Pose2d getTargetPose() {
        Pose2d pose;


        if(s_Swerve.isRedAlliance()) {

            // get pose of red speaker
            pose = new Pose2d(Constants.Positions.hubRedX, Constants.Positions.hubRedY, new Rotation2d(Constants.Positions.hubBlueR));

        // if robot is on blue alliance
        } else {

            // get pose of blue speaker
            pose = new Pose2d(Constants.Positions.hubBlueX, Constants.Positions.hubBlueY, new Rotation2d(Constants.Positions.hubBlueR));

        }
        
        return pose;
     }

     public double getTargetDistance()
     {
        Pose2d robotPose = s_Swerve.m_PoseEstimator.getEstimatedPosition();

        Pose2d targetPose = getTargetPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        double targetX = targetPose.getX();
        double targetY = targetPose.getY();
        double thetaX = targetX - robotX;
        double thetaY = targetY - robotY;
        double distance = Math.sqrt(Math.pow(thetaX, 2) + Math.pow(thetaY, 2));
        return distance;
     }
     
 
     /*
      * This method will return the known pose of the desired target.
      * 
      * parameters:
      * none
      * 
      * returns:
      * target pose      (Pose2d)
      */
     
 
     @Override
     public void periodic() {

        s_Swerve.m_PoseEstimator.update(s_Swerve.getHeading(), s_Swerve.getSwerveDrive().getModulePositions());
        estimatedRobotPosePublisher.set(s_Swerve.m_PoseEstimator.getEstimatedPosition());



        //  if (LimelightHelpers.getTV("limelight-right")) {
        //      s_Swerve.m_PoseEstimator.addVisionMeasurement(
        //          getRobotPose(), 
        //          Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("limelight-right")/1000.0) - (LimelightHelpers.getLatency_Capture("limelight-right")/1000.0)
        //      );
        //  }

        //  if(LimelightHelpers.getTV("limelight-left") == true)
        //  {
        //     s_Swerve.m_PoseEstimator.addVisionMeasurement(
        //          getRobotPoseLeft(), 
        //          Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Pipeline("limelight-left")/1000.0) - (LimelightHelpers.getLatency_Capture("limelight-left")/1000.0)
        //      );
        //  }
 
         updateData();

        SmartDashboard.putNumber("Robot Angle", s_Swerve.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
        SmartDashboard.putNumber("Robot Angle Red", s_Swerve.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees() + 180);
        SmartDashboard.putNumber("Target Angle", getTargetRotation());
        SmartDashboard.putNumber("Velocity Command", (getTargetRotation()-s_Swerve.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees()) * (.1));
        SmartDashboard.putNumber("Velocity Command Red", (getTargetRotation()+(s_Swerve.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees() + 180)) * (-.1));
        SmartDashboard.putNumber("Theta M Negative", -(180 + s_Swerve.m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees()));
        

        targetHub.set(getTargetPose());

 
        
 
 
        
 
     }
 }