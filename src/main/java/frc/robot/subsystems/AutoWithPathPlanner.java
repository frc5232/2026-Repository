// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoWithPathPlanner extends SubsystemBase {
  /** Creates a new AutoWithPathPlanner. */
  PathPlannerAuto mAuto = new PathPlannerAuto("Left blue auto v1");
  AutoBuilder mAutoBuilder = new AutoBuilder();

  public AutoWithPathPlanner() {
    AutoBuilder.buildAuto("left blue auto v1");
    
  }
  public Command getAuto(){
    return mAuto;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
