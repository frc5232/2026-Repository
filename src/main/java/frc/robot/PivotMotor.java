package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;


public class PivotMotor {
    private static TalonFXConfiguration pivotMotorConfig;
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
        this(new TalonFX(99));
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
    /**
     * 
     * @return our pivot motor
     */
    public TalonFX getPivotMotor(){
        return pivotMotorTalon;
    }
    /**
     * 
     * @param kP our proporital to add
     * @param kI our integral to add
     * @param kD our derivative to add
     */
    public void setPidForPivotMotor(double kP,double kI, double kD){
        pivotMotorConfig.Slot0.kP = kP;
        pivotMotorConfig.Slot0.kI = kI;
        pivotMotorConfig.Slot0.kD = kD;
        pivotMotorTalon.getConfigurator().apply(pivotMotorConfig);
    }
    /**
     * 
     * @param kP our proporitonal to add
     * @param kI our integral to add
     * @param kD our derivative to add
     * @param kG our gravitational to add
     * @param kS our static to add
     * @param kV our velocity to add
     * @param kA our acceleration to add
     */
    public void setPidAndkGVSAForPivotMotor(double kP, double kI,double kD, double kG,double kS, double kV, double kA){
        pivotMotorConfig.Slot0.kP = kP;
        pivotMotorConfig.Slot0.kI = kI;
        pivotMotorConfig.Slot0.kD = kD;
        pivotMotorConfig.Slot0.kS = kS;
        pivotMotorConfig.Slot0.kV = kV;
        pivotMotorConfig.Slot0.kA = kA;
        pivotMotorConfig.Slot0.kG = kG;
        pivotMotorTalon.getConfigurator().apply(pivotMotorConfig.Slot0);
    }
}
