// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.derive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Autov2 extends SubsystemBase {
  /** Creates a new Autov2. */
  public CommandSwerveDrivetrain drivetrain;
  PathPlannerAuto mAuto;
  public Autov2(CommandSwerveDrivetrain dCommandSwerveDrivetrain){
    this.drivetrain = dCommandSwerveDrivetrain;
    
  }
  public Command autoBuilder(){
   
   mAuto = pickAndFollowAuto();
   
   return mAuto;
  // return AutoBuilder.buildAuto(mAuto.getName());

  }
  private PathPlannerAuto pickAndFollowAuto(){
    double x = drivetrain.getState().Pose.getX();
    double y = drivetrain.getState().Pose.getY();

    if(x > 7){
      if(y > 5){
        mAuto = new PathPlannerAuto("Left blue part 1");
        
        return mAuto;
      }else if(y < 3){
        mAuto = new PathPlannerAuto("Left blue part 1");
        return mAuto;
      }else{
        mAuto = new PathPlannerAuto("Left blue part 1");
        AutoBuilder.buildAuto(mAuto.getName());
        return mAuto;
      }
    }else{
      if(y > 5){
        mAuto = new PathPlannerAuto("Left blue part 1");
        AutoBuilder.buildAuto(mAuto.getName());
        return mAuto;
      }else if(y < 3){
        mAuto = new PathPlannerAuto("Left blue part 1");
        AutoBuilder.buildAuto(mAuto.getName());
        return mAuto;
      }else{
        mAuto = new PathPlannerAuto("Left blue part 1");
        AutoBuilder.buildAuto(mAuto.getName());
        return mAuto;
      }
    }


  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
