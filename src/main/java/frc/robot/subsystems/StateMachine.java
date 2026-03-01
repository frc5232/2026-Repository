// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateMachine extends SubsystemBase {
  Intake mIntake;
  Shooter mShooter;
  CommandSwerveDrivetrain drivetrain;
  AutoWithPathPlanner mAutoWithPathPlanner;
  public StateMachine(Intake intakeSubsystem, Shooter shooterSubsysem, CommandSwerveDrivetrain drivetrainSubsystem,AutoWithPathPlanner autoWithPathPlanner) {
    this.mIntake = intakeSubsystem;
    this.mShooter = shooterSubsysem;
    this.drivetrain = drivetrainSubsystem;
    this.mAutoWithPathPlanner = autoWithPathPlanner;
    
   
  }
  public void getAuto(){
    new SequentialCommandGroup(mAutoWithPathPlanner.getAuto(1),new InstantCommand(()->mIntake.goToDownPositionCommand()).andThen(mAutoWithPathPlanner.getAuto(2)));//.andThen(mAutoWithPathPlanner.getAuto(3));
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
