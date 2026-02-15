// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

//encoder
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public DutyCycleEncoder mEncoder;
  private boolean startingPosition;
  private TalonFX moveToFloorTalonFX;
  private TalonFX closesTalonFX;
  private double encoderConstant;
  private double encoderDownPos;
  private PIDController mController;
  
  public Intake() {
    
    moveToFloorTalonFX = new TalonFX(8);
    mEncoder = new DutyCycleEncoder(0);
    mController = new PIDController(2, 0, 0.005);
    moveToFloorTalonFX.getConfigurator().apply(Constants.talonIntakeCon.mIntakeMotorConfig);
    
    
    closesTalonFX = new TalonFX(Constants.talonIntakeCon.SPIN_MOTOR_ID);
    // our upper position
    encoderConstant = 0.62;
    // our down positon
    encoderDownPos = 0.93; 
     
  }
  /**
   * our command to go to our down position using a set control until were at our down positon
   */
  private void goToDownPositionCommand(){
    new InstantCommand(()->
     moveToFloorTalonFX.setControl(new MotionMagicExpoVoltage(moveToFloorTalonFX.getPosition().getValueAsDouble() + 0.1)))
     .until(()->atDownPosition());
  }
  /*
   * This is our command to go to our starting pose by chaining our movement to starting pose and saying to do it
   * until startingpose returns true
   */
  private void gotoStartPositonCommand(){
    new InstantCommand(()-> movementToStartingPosition()).until(()-> atStartingPosition());
  }
  /*
   * the background command for the movement for the goToStartPoseCommand so
   * to explain more 
   * if our encoder position is less then our encoder constant which is our up position it moves downward by a tiny bit
   * otherwise if our encoder position is greater then our encoder constant it moves down by a tiny bit
   * use with a .until that way it doesnt keep bouncing back and forth
   */
  private void movementToStartingPosition(){
      if(mEncoder.get() < encoderConstant){
        moveToFloorTalonFX.setControl(new MotionMagicExpoVoltage(moveToFloorTalonFX.getPosition().getValueAsDouble() + 0.1));
      }else if(mEncoder.get() > encoderConstant){
        moveToFloorTalonFX.setControl(new MotionMagicExpoVoltage(moveToFloorTalonFX.getPosition().getValueAsDouble() - 0.1));
      }
    
  }
  /**
   * this is just to publish our encoder position to our smartdashboard
   */
  public void publishPosition(){
    SmartDashboard.putNumber("position", mEncoder.get());
  }
  /*
   * only returns true if encoder is greater then encoder constant and less then encoder constant by 0.03
   */
  private boolean atStartingPosition(){
    if(!(mEncoder.get() < encoderConstant - 0.03) ||!( mEncoder.get() > encoderConstant + 0.03)){
      return false;
    }else{
      return true;
    }
  }
  /*
   * background for the .until for command to go to goal position
   * which works by saying if our encoder positon is greater then the ncoder down position constant
   * - 0.02 it will return true otherwise it will return false
   */
  private boolean atDownPosition(){
    if(mEncoder.get() >= encoderDownPos - 0.02){
      return true;
    }else if(mEncoder.get() <= encoderDownPos - 0.02){
      return false;
    }else{
      return false;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    publishPosition();
    //SmartDashboard.putNumber("position of intake motor", moveToFloorTalonFX.getPosition().getValueAsDouble());
  }
}
