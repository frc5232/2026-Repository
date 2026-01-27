package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Aiming extends SubsystemBase {
  /** Creates a new Aiming. */
  CommandSwerveDrivetrain mDrivetrain;
  FieldCentric drive;
  Pose3d currentPose;
  Pose3d staticHubPose3d;
  public boolean aim = false;
  public Aiming(CommandSwerveDrivetrain drivetrain,SwerveRequest.FieldCentric mCentric) {
    /**
     * Change to actual position of hub pose
     * current pose will just be drivetrain.getstate.pose.get(value)
     */
    staticHubPose3d = new Pose3d(10, 3, 0, new Rotation3d(new Rotation2d(0)));
    currentPose = new Pose3d(13,4,0,new Rotation3d(new Rotation2d(10)));
    drive = mCentric;
    mDrivetrain = drivetrain;
  }
  private void calculate(){
    if(aim){
      double x = Math.abs(currentPose.getX() - staticHubPose3d.getX());
      double y = Math.abs(currentPose.getY() - staticHubPose3d.getY());
      double xx = (x*Math.sin(90))/Math.sqrt((x*x + y*y));
      double yawheading;
      if(DriverStation.getAlliance().get().toString() == "blue"){
        yawheading = 180 + xx;
      }else{
        yawheading = xx;
      }
      new SequentialCommandGroup(mDrivetrain.applyRequest(()->drive.withRotationalRate(4)).until(()-> mDrivetrain.getState().RawHeading.getDegrees() == yawheading ));

  }
    
    
    //mdrivetrain.applyRequest(()-> drive.withRotationalRate(3)).until(mDrivetrain.getstate.pose.getRotation().getAngle() == rotationAmount+xx )
    // adding a closeEnoughType thing so if its within 2 degrees it will be okay
    // make a method constantly called with a bool that is defuealt set to false until a button is pressed which changes it making it so we can turn it on and off
    //then constantly update it so we can drive and aim auto while shooting until disabled
  }
  public void changeBool(){
    aim = !aim;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculate();
  }
}
