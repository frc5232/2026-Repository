// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  // switched this to private right before pushing if it doesnt work next time
  // thats why
  private double goalPositionToTarget = 0;
  private DutyCycleEncoder absoluteEncoder;
  private MotionMagicExpoVoltage mRequest = new MotionMagicExpoVoltage(0);
  private DutyCycleOut mCycleOut = new DutyCycleOut(0);
  private TalonFX moveToFloorTalonFX;
  private TalonFX spinningMotor;
  private double encoderConstant;
  private double encoderDownPos;
  private Trigger startSpinning;

  public Intake() {

    moveToFloorTalonFX = new TalonFX(Constants.talonIntakeCon.INTAKE_MOTOR_ID);

    absoluteEncoder = new DutyCycleEncoder(Constants.talonIntakeCon.ENCODER_ID_CONSTANT);

    moveToFloorTalonFX.getConfigurator().apply(Constants.talonIntakeCon.INTAKE_MOTOR_CONFIG);

    spinningMotor = new TalonFX(Constants.talonIntakeCon.SPIN_MOTOR_ID);

    spinningMotor.getConfigurator().apply(Constants.talonIntakeCon.SPIN_MOTOR_CONFIG);

    encoderConstant = Constants.talonIntakeCon.ENCODER_STARTING_POSITION;

    encoderDownPos = Constants.talonIntakeCon.ENCODER_DOWN_POSITION;
    /*
     * our encoder get minus our constant which is starting position
     * then absolute it to get a positive value
     * then divide it by 0.095(found by testing)
     * to get our position we want
     */
    moveToFloorTalonFX.setPosition(Math.abs(absoluteEncoder.get() - encoderConstant) / 0.095);

    startSpinning = new Trigger(() -> goalPos());

  }

  /**
   * 
   * @param x our goal target position to set (use encoder constants)
   */
  private void setGoal(double x) {
    goalPositionToTarget = x;
  }

  /**
   * 
   * @return our goal target position to get (in the encoder constants)
   */
  private double getGoal() {
    return goalPositionToTarget;
  }

  /**
   * 
   * @return A sequentail command group so we set our goal pose then do our intake
   *         down and spinning
   */
  public Command intakeDown() {
    return new SequentialCommandGroup(new InstantCommand(() -> setGoal(encoderDownPos)).andThen(intakeDownCommand()));
  }

  /**
   * 
   * @return A sequentail command group so we set our goal pose then do our intake
   *         up and stop spinning
   */
  public Command intakeUp() {
    return new SequentialCommandGroup(new InstantCommand(() -> setGoal(encoderConstant)).andThen(intakeUpCommand()));
  }

  /**
   * 
   * @return A command in parallel to run our motors
   */
  private Command intakeDownCommand() {
    /*
     * Test deadline v Parrellel
     */

    return Commands.parallel(movingMotor(81), spinMotor(0.32));
  }

  /**
   * parallel commands to run until they are both done
   * 
   * @return A command
   */
  private Command intakeUpCommand() {
    // setGoal(encoderConstant);
    return Commands.parallel(movingMotor(0), spinMotor(0));

  }

  /**
   * 
   * @param dutyCycleAmount our duty cycle amount we want the motor spinning by
   *                        (Between 1 and -1)
   * @return A Instant Command to spin the motor
   */
  private Command spinMotor(double dutyCycleAmount) {
    return new InstantCommand(() -> spinningMotor.setControl((mCycleOut.withOutput(dutyCycleAmount))));
  }

  /**
   * 
   * @param goalAmount The goal amount in degrees you want the motor to move by
   * @return A functional command to move the motor with
   */
  private Command movingMotor(double goalAmount) {

    return new FunctionalCommand(
        () -> {
        },
        () -> moveToFloorTalonFX.setControl(mRequest.withPosition(Units.degreesToRotations(goalAmount) * 26.64)),

        startSpinning -> {
          if (startSpinning == true) {
            moveToFloorTalonFX.stopMotor();
          }
        },
        () -> goalPos(),
        this);

    // **** Try This ////////////////////////////

    // return new FunctionalCommand(
    // // OnInit
    // () -> moveToFloorTalonFX.setControl(mRequest.withPosition(goalAmount)),

    // // OnExecute
    // () -> {},

    // //OnEnd
    // interrupted -> moveToFloorTalonFX.stopMotor(),

    // // isFinished

    // () -> goalPos(),

    // this
    // );

    ///////////////////////////////////////////////////////
  }

  /**
   * 
   * @return A Boolean to say if were near our goal pos
   */
  private boolean goalPos() {

    if (isNear(getGoal(), absoluteEncoder.get(), 0.03)) {
      return true;
    }
    return false;

  }

  private boolean isNear(double valueToCheck, double checkingValue, double tolerance) {
    return Math.abs(valueToCheck - checkingValue) <= tolerance;

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoder value", absoluteEncoder.get());
    SmartDashboard.putNumber("Position of Motor", moveToFloorTalonFX.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Spin motor speed", spinningMotor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber("goal target", getGoal());
  }

}
