// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.List;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Auto extends SubsystemBase {
  /** Creates a new Auto. */
  
  private Pose2d goalPose2d;
  private Pose2d currentPose2d;
  private SwerveRequest.FieldCentric drive;
  private CommandSwerveDrivetrain mDrivetrain;
  private Trajectory mTrajectory;
  private TrajectoryConfig mConfig;
  private LTVUnicycleController mController;
  private ChassisSpeeds m_ChassisSpeeds;
  public Auto(CommandSwerveDrivetrain drivetrain,SwerveRequest.FieldCentric mCentric) {
    mDrivetrain = drivetrain;
    drive = mCentric;
    currentPose2d = mDrivetrain.getState().Pose;



    mTrajectory = new Trajectory();
    // we can tune this some more when we are at arc field
    mController = new LTVUnicycleController(VecBuilder.fill(0.0625, 0.125, 2.0), VecBuilder.fill(1.0, 2.0), 0.02, 9);
    PickAutoToRun();
  }
  /**
   * Our method to update our pose and needs to be called in peridioic
   */
  private void updatePose(){
    currentPose2d = mDrivetrain.getState().Pose;
  }
  /**
  * @param c - is our current pose - most likely pass current vision measurement into it
  * @param g - is our goal pose
  */
  private Trajectory trejGen(Pose2d c, Pose2d g){
    mConfig = new TrajectoryConfig(5, 2);
    /**
     * might end up having the list of get passed in but for now we are just having the point to go through be the starting point
     */
    mTrajectory = TrajectoryGenerator.generateTrajectory(c, List.of(new Translation2d(c.getX(),c.getY())), g, mConfig);
    return mTrajectory;
  }
  /**
   * 
   * @param tj our trajectory generated at the start of auto by pick auto to run
   * @param cont our controller as defined in method header
   * @return a chassis speeds which is called in peridoic to update the speeds and rotations of our drivetrain
   */
  private ChassisSpeeds mGenSpeeds(Trajectory tj,LTVUnicycleController cont){
    
    m_ChassisSpeeds = cont.calculate(currentPose2d, tj.sample(tj.getTotalTimeSeconds())); 
    return m_ChassisSpeeds;
  }
  /**
   * 
   * @param mSpeeds our chassis speeds getting passed into the drivetrain
   */
  private void followAuto(ChassisSpeeds mSpeeds){
    /**
     * Making chassisSpeeds using our controller and how far we are in the trajectory
     * Might want to be updated constantly
     * Make it so traj calc and command group is diffrent methods then when we are close enough to goal pose it just skips over it using if statement
     * 
     */

    new SequentialCommandGroup(mDrivetrain.applyRequest(()-> drive.withVelocityX(mSpeeds.vxMetersPerSecond)).until(()->checking(currentPose2d, goalPose2d,'x') == true)
    .alongWith(mDrivetrain.applyRequest(()->drive.withVelocityY(mSpeeds.vyMetersPerSecond))).until(()->checking(currentPose2d, goalPose2d, 'y'))
    .alongWith(mDrivetrain.applyRequest(()->drive.withRotationalRate(mSpeeds.omegaRadiansPerSecond)).until(()->checking(currentPose2d, goalPose2d, 'r'))));
   
  }
  /**
   * picks which auto to run
   * @return our auto to run which is just basically our goal pose
   * Notes:
   * value for rotation will need to be adjusted
   * aswell as pose with testing
   * this mainly just needs a lot of setting values to work and thats it
   * just deleted a besic for what they will look like becasue i redid the trajectory gen and following
   */

  public Pose2d PickAutoToRun(){
       return new Pose2d(0,0,new Rotation2d(0));
    }
   

  /**
   * 
   * @param c - the current pose
   * @param g - the goal pose
   * @param typeToCheck - the type to check - x for x value, y for y value, r for rotation
   * @return returns a bool which decides whether or whether not to stop
   */
  private Boolean checking(Pose2d c, Pose2d g,char typeToCheck){
    if(typeToCheck == 'x'){
    if(c.getX() > g.getX() - 0.25){
      return true;
    }}else if(typeToCheck == 'y'){
      if(c.getY() > g.getY() - 0.25){
      return true;
    }}else if (typeToCheck == 'r') {
      if (c.getRotation().getDegrees() > g.getRotation().getDegrees() -3) {
        return true;
      }
    }
    return false;
    }
    /**
     * 
     * @param x the x coord we want to line up with and we drive towards it will a speed of 0.5m/s and go till were within 1% of our goal
     * @param y the y coord we want to line up with and we drive towards it will a speed of 0.5m/s and go till were within 1% of our goal
     *It does a sequental command group where it drives with velocity until it is near the goal coords with a tolerance of 1%
     */
  private void alignWithClimb(double x, double y){
    new SequentialCommandGroup(mDrivetrain.applyRequest(()-> drive.withVelocityX(0.5)).until(()-> mDrivetrain.getState().Pose.getMeasureX().isNear(Units.Meters.of(x), 0.01))
    .andThen(mDrivetrain.applyRequest(()->drive.withVelocityY(0.5)).until(()-> mDrivetrain.getState().Pose.getMeasureY().isNear(Units.Meters.of(y), 0.01))));
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose();
    if(DriverStation.isAutonomous()){
      followAuto(mGenSpeeds(mTrajectory, mController));
    }
  }
}
