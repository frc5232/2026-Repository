package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Aiming extends SubsystemBase {
  /** Creates a new Aiming. */
  CommandSwerveDrivetrain mDrivetrain;
  FieldCentric drive;
  Pose2d currentPose;
  Pose2d staticHubPose2d;
  double yawheading;
  public boolean aim = false;
  public Aiming(CommandSwerveDrivetrain drivetrain,SwerveRequest.FieldCentric mCentric) {
    /**
     * Change to actual position of hub pose
     * current pose will just be drivetrain.getstate.pose.get(value)
     */
    mDrivetrain = drivetrain;
    pickHubPose();
    currentPose = mDrivetrain.getState().Pose;
    drive = mCentric;
    
  }
  private void calculate(){
    if(aim){
      // doubls is our pose - hub pose 
      double x = Math.abs(currentPose.getX() - staticHubPose2d.getX());
      double y = Math.abs(currentPose.getY() - staticHubPose2d.getY());
      // doing a angle calulation
      double xx = (x*Math.sin(90))/Math.sqrt((x*x + y*y));
      
      // if((currentPose.getY() < 4 && currentPose.getX() > 7) || (currentPose.getY() > 4 && currentPose.getX() < 7)){
      //     xx = -xx;
      //   }
      //for blue alliance
      if(currentPose.getX() > 7){
        if(currentPose.getY() <4){
          // for away from scoring table
          yawheading = xx;
        }//close to scoring table
        else{
          yawheading = 360 - xx;
        }
        
      }// for our red alliance 
      else if(currentPose.getX() < 7){
        // if closer to scoring table
        if(currentPose.getY() <4){
           yawheading = 180 - xx;
        }// if away from scoring table
        else{
          yawheading = 180 + xx;  
        }}
    
      
      new SequentialCommandGroup(mDrivetrain.applyRequest(()->drive.withRotationalRate(4))
      .until(()-> yawheadChecker(mDrivetrain.getState().RawHeading.getDegrees(), yawheading)));

  }
    
    
    
    
    //then constantly update it so we can drive and aim auto while shooting until disabled
  }
  /**
   * hook up to a button so we can pick if we want to aim
   */
  private void changeAimState(){
    aim = !aim;
  }
  private void pickHubPose(){
    if(DriverStation.getAlliance().get().toString() == "red"){
      staticHubPose2d = new Pose2d(Units.inchesToMeters(469.11),Units.inchesToMeters(159.1),new Rotation2d(0));
    }else{
      staticHubPose2d = new Pose2d(Units.inchesToMeters(182.11),Units.inchesToMeters(159.1),new Rotation2d(0));
    }
  }
  private boolean yawheadChecker(double cY, double yaw){
    if(cY +2 >= yaw || cY - 2 <= yaw){
      return true;
    }
    yawheadChecker(cY, yaw);
    return false;
  }
  private void updateCurrentPose(){
    currentPose = mDrivetrain.getState().Pose;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculate();
    updateCurrentPose();
  }
}
