package frc.robot;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;


public class PivotMotor {
    private static TalonFX pivotMotorTalon;
    /**
     * 
     * @param pivotMotor your pivot motor which is a talonFx
     */
    public PivotMotor(TalonFX pivotMotor){
        pivotMotorTalon = pivotMotor;
    }
    /**
     * Default constructor that assigns id of zero to the talonFx
     */
    public PivotMotor(){
        this(new TalonFX(0));
    }
    /**
     * 
     * @param degreesToGoTo The degrees to target with your PivotMotor
     * Using motion magic means you have it fully tuned with kP,kI,kD,kS,kV,kG, and kA
     * 
     */
    public static void goToPositionInDegreesUsingMotionMagic(double degreesToGoTo){
        pivotMotorTalon.setControl(new MotionMagicExpoVoltage(Units.degreesToRotations(degreesToGoTo)));
    }
    /**
     * 
     * @param degreesToGoTo The degrees to target with your PivotMotor
     * You dont have to have it fully tuned for this but you should have PID tuned at least partially
     */
    public static void goToPositionInDegreesUsingPositionVoltage(double degreesToGoTo){
        pivotMotorTalon.setControl(new PositionVoltage(Units.degreesToRotations(degreesToGoTo)));
    }
    public TalonFX getPivotMotor(){
        return pivotMotorTalon;
    }
}
