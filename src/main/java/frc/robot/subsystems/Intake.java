// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
//encoder
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private Consumer<Boolean> m_blankBool;
  public DutyCycleEncoder absoluteEncoder;
  private MotionMagicExpoVoltage mRequest = new MotionMagicExpoVoltage(0);
  private TalonFX moveToFloorTalonFX;
  private TalonFX spinningMotor;
  private double encoderConstant;
  private double encoderDownPos;
  private double encoderPosition;

  public Intake() {
    moveToFloorTalonFX = new TalonFX(8);
    absoluteEncoder = new DutyCycleEncoder(0);

    moveToFloorTalonFX.getConfigurator().apply(Constants.talonIntakeCon.INTAKE_MOTOR_CONFIG);

    spinningMotor = new TalonFX(Constants.talonIntakeCon.SPIN_MOTOR_ID);
    spinningMotor.getConfigurator().apply(Constants.talonIntakeCon.SPIN_MOTOR_CONFIG);

    encoderConstant = Constants.talonIntakeCon.ENCODER_STARTING_POSITION;

    encoderDownPos = 0.32;
    
   // moveToFloorTalonFX.setPosition(Units.degreesToRotations(90*absoluteEncoder.get()));

  }

  public Command intakeDownCommand() {
    return Commands.deadline(movingMotor(90), spinMotor());
  }

  private Command spinMotor() {
    return new InstantCommand(() -> spinningMotor.setControl(new DutyCycleOut(1)));
  }

  private Command movingMotor(double goalAmount) {
    return new FunctionalCommand(() -> moveToFloorTalonFX.setControl(mRequest.withPosition(Units.rotationsToDegrees(goalAmount))),()->{}, m_blankBool, ()->goalPos(), this);

    }

  public boolean goalPos() {
    if (absoluteEncoder.get() >= 0.3 && absoluteEncoder.get() <= 0.33) {
      return true;
    } else {
      return false;
    }
  }
  private Command onEndBlankCommand(){
    return new InstantCommand();
  }

  @Override
  public void periodic() {

  }

}
