// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

//encoder
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public DutyCycleEncoder absoluteEncoder;
  private boolean startingPosition;
  private TalonFX moveToFloorTalonFX;
  private TalonFX closesTalonFX;
  private double encoderConstant;
  private double encoderDownPos;
  private PIDController pidController;

  public Intake() {

    moveToFloorTalonFX = new TalonFX(8);
    absoluteEncoder = new DutyCycleEncoder(0);
    
    moveToFloorTalonFX.getConfigurator().apply(Constants.talonIntakeCon.INTAKE_MOTOR_CONFIG);

    closesTalonFX = new TalonFX(Constants.talonIntakeCon.SPIN_MOTOR_ID);
    // our upper position
    encoderConstant = 0.62;
    // our down positon
    encoderDownPos = 0.93;

  }

  /**
   * our command to go to our down position using a set control until were at our
   * down positon
   */
  public Command goToDownPositionCommand() {
    return new InstantCommand(() -> moveToFloorTalonFX
        .setControl(new MotionMagicExpoVoltage(moveToFloorTalonFX.getPosition().getValueAsDouble() + 0.1)));
        //.until(() -> atDownPosition())
  }

  /*
   * This is our command to go to our starting pose by chaining our movement to
   * starting pose and saying to do it
   * until startingpose returns true
   */
  public Command gotoStartPositonCommand() {
    return new InstantCommand(() -> moveToFloorTalonFX
          .setControl(new PositionVoltage(moveToFloorTalonFX.getPosition().getValueAsDouble() - 0.1)));
    //.until(() -> atStartingPosition()
  }

  /*
   * the background command for the movement for the goToStartPoseCommand so
   * to explain more
   * if our encoder position is less then our encoder constant which is our up
   * position it moves downward by a tiny bit
   * otherwise if our encoder position is greater then our encoder constant it
   * moves down by a tiny bit
   * use with a .until that way it doesnt keep bouncing back and forth
   */
  private void movementToStartingPosition() {
    if (absoluteEncoder.get() < encoderConstant) {
      moveToFloorTalonFX
          .setControl(new MotionMagicExpoVoltage(moveToFloorTalonFX.getPosition().getValueAsDouble() + 0.1));
    } else if (absoluteEncoder.get() > encoderConstant) {
      
    }

  }

  /**
   * this is just to publish our encoder position to our smartdashboard
   */
  public void publishPosition() {
    SmartDashboard.putNumber("position", absoluteEncoder.get());
  }

  /*
   * only returns true if encoder is greater then encoder constant and less then
   * encoder constant by 0.03
   */
  private boolean atStartingPosition() {
    if (!(absoluteEncoder.get() < encoderConstant - 0.03) || !(absoluteEncoder.get() > encoderConstant + 0.03)) {
      return false;
    } else {
      return true;
    }
  }

  /*
   * background for the .until for command to go to goal position
   * which works by saying if our encoder positon is greater then the ncoder down
   * position constant
   * - 0.02 it will return true otherwise it will return false
   */
  private boolean atDownPosition() {
    if (absoluteEncoder.get() >= encoderDownPos - 0.02) {
      return true;
    } else if (absoluteEncoder.get() <= encoderDownPos - 0.02) {
      return false;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    publishPosition();
    // SmartDashboard.putNumber("position of intake motor",
    // moveToFloorTalonFX.getPosition().getValueAsDouble());
  }
}
