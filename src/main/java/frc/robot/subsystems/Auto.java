// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.ArrayList;
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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.Units;
public class Auto extends SubsystemBase {
  /** Creates a new Auto. */
  private boolean firstRunOfTraj = false;
  private boolean autoHasBeenPicked;
  private double xForClimb;
  private double yForClimb;
  private Pose2d goalPose2d;
  private Pose2d currentPose2d;
  private SwerveRequest.FieldCentric drive;
  private CommandSwerveDrivetrain mDrivetrain;
  private Trajectory mTrajectory;
  private TrajectoryConfig mConfig;
  private LTVUnicycleController mController;
  private ChassisSpeeds m_ChassisSpeeds;
  private ArrayList<Double> xValueForWayPoints = new ArrayList<>();
  private ArrayList<Double> yValueForWayPoints = new ArrayList<>();
 
  public Auto(CommandSwerveDrivetrain drivetrain,SwerveRequest.FieldCentric mCentric) {
    mDrivetrain = drivetrain;
    drive = mCentric;
    currentPose2d = mDrivetrain.getState().Pose;

    

    mTrajectory = new Trajectory();
    // we can tune this some more when we are at arc field
    // this is for path of the auto from start to where we want to be
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
    
    // for(int i =0; i < xValueForWayPoints.size();i++){
    //   mList.add(i, new Translation2d(xValueForWayPoints.get(i),yValueForWayPoints.get(i)));
    // }
    mTrajectory = TrajectoryGenerator.generateTrajectory(c,List.of(new Translation2d(xValueForWayPoints.get(0),yValueForWayPoints.get(0))), g, mConfig);
    return mTrajectory;
  }
  /**
   * 
   * @param tj our trajectory generated at the start of auto by pick auto to run
   * @param cont our controller as defined in method header
   * @return a chassis speeds which is called in peridoic to update the speeds and rotations of our drivetrain
   */
  private ChassisSpeeds mGenSpeeds(Trajectory tj,LTVUnicycleController cont){
    if(firstRunOfTraj){
    m_ChassisSpeeds = cont.calculate(currentPose2d, tj.sample(tj.getTotalTimeSeconds())); 
    return m_ChassisSpeeds;}
      m_ChassisSpeeds = cont.calculate(currentPose2d,tj.sample(0) );
      firstRunOfTraj = true; 
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
    if(checking(currentPose2d, goalPose2d, 'x')){
      mSpeeds.vxMetersPerSecond = 0;
    }
    if(checking(currentPose2d, goalPose2d, 'y')){
      mSpeeds.vyMetersPerSecond = 0;
    }
    if(checking(currentPose2d, goalPose2d, 'r')){
      mSpeeds.omegaRadiansPerSecond = 0;
    }
    new SequentialCommandGroup(new InstantCommand(()->mDrivetrain.applyRequest(()->drive.withVelocityX(mSpeeds.vxMetersPerSecond)))
    .alongWith(new InstantCommand(()-> mDrivetrain.applyRequest(()-> drive.withVelocityY(mSpeeds.vyMetersPerSecond)))
    .alongWith(new InstantCommand(()-> mDrivetrain.applyRequest(()->drive.withRotationalRate(mSpeeds.omegaRadiansPerSecond))))));
    
   
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
    //for blue
    
    //for red
    
    if(currentPose2d.getX() > 7){
      // if(currentPose2d.getY() > 5){

      // }else if(currentPose2d.getY() < 3){

      // }
      
       goalPose2d = new Pose2d(edu.wpi.first.math.util.Units.inchesToMeters(651),edu.wpi.first.math.util.Units.inchesToMeters(124),new Rotation2d(180));
        //mList.add(0, new Translation2d(goalPose2d.getX() + 0.25,goalPose2d.getY() + 0.25));
        xValueForWayPoints.add(0,goalPose2d.getX() + 0.5);
        yValueForWayPoints.add(0,goalPose2d.getY() + 0.5);
      }else{
      // if(currentPose2d.getY() > 5){

      // }else if(currentPose2d.getY() < 3){
        
      // }
      goalPose2d = new Pose2d(edu.wpi.first.math.util.Units.inchesToMeters(46.5),edu.wpi.first.math.util.Units.inchesToMeters(146.86),new Rotation2d(0));
         //mList.add(0, new Translation2d(goalPose2d.getX() + 0.25,goalPose2d.getY() + 0.25));
         xValueForWayPoints.add(0,goalPose2d.getX() + 0.5);
        yValueForWayPoints.add(0,goalPose2d.getY() + 0.5);
    }
       autoHasBeenPicked = true;
       trejGen(currentPose2d, goalPose2d);
       return goalPose2d;
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
     *42 inches away from x coord
     *blue 43,147
     *red 606,170
     */
  private void alignWithClimb(){
    
    if(DriverStation.getAlliance().get().toString() == "blue"){
      xForClimb = edu.wpi.first.math.util.Units.inchesToMeters(45);
      yForClimb = edu.wpi.first.math.util.Units.inchesToMeters(147);
    }else if(DriverStation.getAlliance().get().toString() == "red"){
       xForClimb = edu.wpi.first.math.util.Units.inchesToMeters(606);
       yForClimb = edu.wpi.first.math.util.Units.inchesToMeters(170);
    }
    new SequentialCommandGroup(mDrivetrain.applyRequest(()-> drive.withVelocityX(0.5)).until(()-> mDrivetrain.getState().Pose.getMeasureX().isNear(Units.Meters.of(xForClimb), 0.01))
    .andThen(mDrivetrain.applyRequest(()->drive.withVelocityY(0.5)).until(()-> mDrivetrain.getState().Pose.getMeasureY().isNear(Units.Meters.of(yForClimb), 0.01))));
  }
  /**
   * Checking our goal pose so that way it wont keep trying to drive towards a goal if its close enough to it
   * mainly for handling and making it easier to use
   * I dont even think its needed but just in case
   */
  private void goalPoseChecking(){
    Pose2d m = mDrivetrain.getState().Pose;
    double x = m.getX();
    double y = m.getY();  
    
    double dx = goalPose2d.getX();
    double dy = goalPose2d.getY();
    Rotation2d dR = goalPose2d.getRotation();
    if(m.getMeasureX().isNear(goalPose2d.getMeasureX(), 0.03)){
      
      goalPose2d = new Pose2d(x,dy,dR);
      dx = goalPose2d.getX();
      dy = goalPose2d.getY();
      dR = goalPose2d.getRotation();
    }else if(m.getMeasureY().isNear(goalPose2d.getMeasureY(), 0.03)){
      goalPose2d = new Pose2d(dx,y,dR);
      dx = goalPose2d.getX();
      dy = goalPose2d.getY();
      dR = goalPose2d.getRotation();
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose();
    if(DriverStation.isAutonomous() && autoHasBeenPicked == true){
      followAuto(mGenSpeeds(mTrajectory, mController));
    }
   
  }
}
