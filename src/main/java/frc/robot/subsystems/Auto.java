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
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
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
  private ChassisSpeeds mChassisSpeeds;
  private DriverStationSim mDriverStationSim;
  public Auto(CommandSwerveDrivetrain drivetrain,SwerveRequest.FieldCentric mCentric) {
    mDrivetrain = drivetrain;
    drive = mCentric;
    currentPose2d = mDrivetrain.getState().Pose;
    mTrajectory = new Trajectory();
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
    mTrajectory = TrajectoryGenerator.generateTrajectory(c, List.of(new Translation2d(c.getX()+0.25,c.getY()+0.25)), g, mConfig);
    return mTrajectory;
  }
  /**
   * 
   * @param tj our trajectory we generated
   * @param cont a Ltv contoller defined and created within our main
   */
  private boolean followAuto(Trajectory tj,LTVUnicycleController cont){
    mChassisSpeeds = cont.calculate(currentPose2d, tj.sample(tj.getTotalTimeSeconds()));
    new SequentialCommandGroup(mDrivetrain.applyRequest(()-> drive.withVelocityX(mChassisSpeeds.vxMetersPerSecond)).until(()->checking(currentPose2d, goalPose2d,'x') == true)
    .alongWith(mDrivetrain.applyRequest(()->drive.withVelocityY(mChassisSpeeds.vyMetersPerSecond))).until(()->checking(currentPose2d, goalPose2d, 'y'))
    .alongWith(mDrivetrain.applyRequest(()->drive.withRotationalRate(mChassisSpeeds.omegaRadiansPerSecond)).until(()->checking(currentPose2d, goalPose2d, 'r'))));
    return true;
  }
  /**
   * picks which auto to run
   * @return our auto to run which is just basically our goal pose
   * Notes:
   * value for rotation will need to be adjusted
   * aswell as pose with testing
   */

  public Pose2d PickAutoToRun(){
    if(currentPose2d.getX() > 7){
      if(currentPose2d.getY() < 3){
        currentPose2d = mDrivetrain.getState().Pose;
        goalPose2d = new Pose2d(13, 3, new Rotation2d(0));
       
      }else if(currentPose2d.getY() > 5){
        currentPose2d = mDrivetrain.getState().Pose;
        goalPose2d = new Pose2d(2, 4, new Rotation2d(0));
        
      }else{
        /**
         * This is what all other autos will look like before arc field
         */
        currentPose2d = mDrivetrain.getState().Pose;
        goalPose2d = new Pose2d(11, 4, new Rotation2d(0));
        if(followAuto(trejGen(currentPose2d, goalPose2d),mController)){
          currentPose2d = mDrivetrain.getState().Pose;
          goalPose2d = new Pose2d(13,4,new Rotation2d(0));
          followAuto(trejGen(currentPose2d, goalPose2d),mController);
          alignWithClimb(13, 4.2);
        };

      }
      }else{
      if(currentPose2d.getY() < 3){
        currentPose2d = mDrivetrain.getState().Pose;
        goalPose2d = new Pose2d(2, 4, new Rotation2d(0));
        
      }else if(currentPose2d.getY() > 5){
        currentPose2d = mDrivetrain.getState().Pose;
        goalPose2d = new Pose2d(2, 4, new Rotation2d(0));
        
      }else{
        currentPose2d = mDrivetrain.getState().Pose;
        
        goalPose2d = new Pose2d(2, 4, new Rotation2d(0));}
      }
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
     * Does a sequentail command group which apllies a velocity towards the x until its within 0.15m of 15m of the x coord on the field
     * then it does a velocity y till were close to 4.2
     */
  private void alignWithClimb(double x, double y){
    new SequentialCommandGroup(mDrivetrain.applyRequest(()-> drive.withVelocityX(0.5)).until(()-> mDrivetrain.getState().Pose.getMeasureX().isNear(Units.Meters.of(x), 0.01))
    .andThen(mDrivetrain.applyRequest(()->drive.withVelocityY(0.5)).until(()-> mDrivetrain.getState().Pose.getMeasureY().isNear(Units.Meters.of(y), 0.01))));
  
  }
  private void simulationing(){
    PickAutoToRun();

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose();
  }
}
