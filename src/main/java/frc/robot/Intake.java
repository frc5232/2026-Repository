package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;

public class Intake {
    private static ArrayList<SpinMotor> arrayOfIntakeSpinMotors;
    private static ArrayList<PivotMotor> arrayOfPivotMotors;
    private static ArrayList<String> arrayOfIntakeSpinMotorNames;
    private static ArrayList<Integer> arrayOfIntakeSpinMotorIds;
    private static ArrayList<String> arrayOfIntakePivotMotorNames;
    private static ArrayList<Integer> arrayOfIntakePivotMotorIds;

    private static Throwable addingThrowable = new Throwable("Wrong type of motor it should be TalonFX");
    private static Throwable removingThrowable = new Throwable("No motor found to remove");
    /**
     * 
     * @param spinMotor Our TalonFX spin motor
     * @throws Throwable A throwable if it is wrong motor type
     */
    public static void addASpinMotor(TalonFX spinMotor) throws Throwable {
        if (spinMotor.equals(new TalonFX(spinMotor.getDeviceID()))) {
            arrayOfIntakeSpinMotors.add(new SpinMotor(spinMotor));
            arrayOfIntakeSpinMotorNames.add(spinMotor.getDescription());
            arrayOfIntakeSpinMotorIds.add(spinMotor.getDeviceID());
        }
            throw addingThrowable;
        
    }
    /**
     * 
     * @param listOfPivotMotors A ArrayList of TalonFxs to add 
     * @throws Throwable a Throwable with an error message of which isnt a talonFx
     */
    public static void addMultiplePivotMotors(ArrayList<TalonFX> listOfPivotMotors) throws Throwable {

        for (int i = 0; i < listOfPivotMotors.size(); i++) {
            if (listOfPivotMotors.get(i).equals(new TalonFX(listOfPivotMotors.get(i).getDeviceID()))) {
                arrayOfPivotMotors.add(new PivotMotor(listOfPivotMotors.get(i)));
                arrayOfIntakePivotMotorIds.add(listOfPivotMotors.get(i).getDeviceID());
                arrayOfIntakePivotMotorNames.add(listOfPivotMotors.get(i).getDescription());
            } else {
                Throwable mThrowable = new Throwable("Wrong pivot motor type at " + i, addingThrowable);
                throw mThrowable;
            }

        }
    }
    /**
     * 
     * @param listOfSpinMotors an ArrayList of TalonFx to be our spin motors
     * @throws Throwable a throwable of which isnt a TalonFx
     */
    public static void addMultipleSpinMotors(ArrayList<TalonFX> listOfSpinMotors) throws Throwable {
        for (int i = 0; i < listOfSpinMotors.size(); i++) {
            if (listOfSpinMotors.get(i).equals(new TalonFX(listOfSpinMotors.get(i).getDeviceID()))) {
                arrayOfIntakeSpinMotors.add(new SpinMotor(listOfSpinMotors.get(i)));
                arrayOfIntakeSpinMotorIds.add(listOfSpinMotors.get(i).getDeviceID());
                arrayOfIntakeSpinMotorNames.add(listOfSpinMotors.get(i).getDescription());
            } else {
                Throwable mThrowable = new Throwable("Wrong spin motor type at " + i, addingThrowable);
                throw mThrowable;
            }
        }
    }
    /**
     * 
     * @param pivotMotor a TalonFx to be turned into a pivot motor
     * @throws Throwable a throwable error message of how its wrong
     */
    public static void addAPivotMotor(TalonFX pivotMotor) throws Throwable {
        if (pivotMotor.equals(new TalonFX(pivotMotor.getDeviceID()))) {
            arrayOfPivotMotors.add(new PivotMotor(pivotMotor));
            arrayOfIntakePivotMotorIds.add(pivotMotor.getDeviceID());
            arrayOfIntakePivotMotorNames.add(pivotMotor.getDescription());
        }
        throw addingThrowable;
    }
    /**
     * 
     * @param pivotMotorId pivot motor Id to get
     * @return the first pivot motor in the list with that id
     * @throws Throwable if none have that id
     */
    public static TalonFX getAPivotMotor(int pivotMotorId) throws Throwable {
        Throwable eThrowable = new Throwable("Does not exist within object\n Error at method :" + checkWhichMethod(1));
        for (int i = 0; i < arrayOfPivotMotors.size(); i++) {
            if (arrayOfPivotMotors.get(i).getPivotMotor().getDeviceID() == pivotMotorId) {
                return arrayOfPivotMotors.get(i).getPivotMotor();
            }
        }
        throw eThrowable;
    }
    /**
     * 
     * @param name the name of the motor to get
     * @return the first pivot motor in the intake with that name
     * @throws Throwable if none have that name throws error
     */
    public static TalonFX getAPivotMotor(String name) throws Throwable {
        Throwable eThrowable = new Throwable("Does not exist within object\n Error at method :" + checkWhichMethod(1));
        for (int i = 0; i < arrayOfPivotMotors.size(); i++) {
            if (arrayOfPivotMotors.get(i).getPivotMotor().getDescription().toLowerCase() == name.toLowerCase()) {
                return arrayOfPivotMotors.get(i).getPivotMotor();
            }
        }
        throw eThrowable;
    }
    /**
     * 
     * @param spinMotorId our spin motor id to get
     * @return the first spin motor in our intake with that id
     * @throws Throwable if none have that id it throws a error
     */
    public static TalonFX getASpinMotor(int spinMotorId) throws Throwable {
        Throwable eThrowable = new Throwable("Does not exist within object\n Error at method :" + checkWhichMethod(2));
        for (int i = 0; i < arrayOfIntakeSpinMotors.size(); i++) {
            if (arrayOfIntakeSpinMotors.get(i).getSpinMotor().getDeviceID() == spinMotorId) {
                return arrayOfIntakeSpinMotors.get(i).getSpinMotor();
            }
        }
        throw eThrowable;
    }
    /**
     * 
     * @param name name of our spin motor
     * @return the first spin motor in intake with that name
     * @throws Throwable if none have that name then throw error
     */
    public static TalonFX getASpinMotor(String name) throws Throwable {
        Throwable eThrowable = new Throwable("Does not exist within object\n Error at method :" + checkWhichMethod(2));
        for (int i = 0; i < arrayOfIntakeSpinMotors.size(); i++) {
            if (arrayOfIntakeSpinMotors.get(i).getSpinMotor().getDescription().toLowerCase() == name.toLowerCase()) {
                return arrayOfIntakeSpinMotors.get(i).getSpinMotor();
            }
        }

        throw eThrowable;
    }
    /**
     * 
     * @return A ArrayList of all the spin motors
     */
    public static ArrayList<TalonFX> getAllSpinMotors() {
        ArrayList<TalonFX> mList = new ArrayList<TalonFX>();
        for(int i = 0; i < arrayOfIntakeSpinMotors.size();i++){
            mList.add(arrayOfIntakeSpinMotors.get(i).getSpinMotor());
        }
        return mList;
    }
    /**
     * 
     * @return a arraylist of all our pivot motors
     */
    public static ArrayList<TalonFX> getAllPivotMotors() {
        ArrayList<TalonFX> mList = new ArrayList<TalonFX>();
        for(int i = 0; i < arrayOfPivotMotors.size();i++){
            mList.add(arrayOfPivotMotors.get(i).getPivotMotor());
        }
        return mList;
    }
    /**
     * 
     * @param name the name of the pivot motor to remove
     * @throws Throwable if none are foudn with that name throws a error
     */
    public static void removePivotMotor(String name) throws Throwable{
        Throwable eThrowable = new Throwable( removingThrowable.getMessage() + "At method:" + checkWhichMethod(4));
        for (int i = 0; i < arrayOfPivotMotors.size(); i++) {
            if (arrayOfPivotMotors.get(i).getPivotMotor().getDescription().toLowerCase() == name.toLowerCase()) {
                arrayOfPivotMotors.remove(i);
                arrayOfIntakePivotMotorIds.remove(i);
                arrayOfIntakePivotMotorIds.remove(i);
                break;
            }
        }
        
        throw eThrowable;
        
    }
    /**
     * 
     * @param id the id of the pivot motor to remove
     * @throws Throwable if none have id throws a error
     */
    public static void removePivotMotor(int id) throws Throwable{
        Throwable eThrowable = new Throwable(removingThrowable.getMessage() + checkWhichMethod(4));
        for (int i = 0; i < arrayOfPivotMotors.size(); i++) {
            if (arrayOfPivotMotors.get(i).getPivotMotor().getDeviceID() == id) {
                arrayOfPivotMotors.remove(i);
                arrayOfIntakePivotMotorIds.remove(i);
                arrayOfIntakePivotMotorIds.remove(i);
                break;
            }
        }
        throw eThrowable;
        
    }
    /**
     * 
     * @param id the id of the spin motor to remove
     */
    public static void removeSpinMotor(int id) throws Throwable{
        Throwable eThrowable = new Throwable(removingThrowable.getMessage() + checkWhichMethod(3));
        for (int i = 0; i < arrayOfIntakeSpinMotors.size(); i++) {
            if (arrayOfIntakeSpinMotors.get(i).getSpinMotor().getDeviceID() == id) {
                arrayOfIntakeSpinMotorIds.remove(i);
                arrayOfIntakeSpinMotorNames.remove(i);
                arrayOfIntakeSpinMotors.remove(i);
                break;
            }
        }
        throw eThrowable;
    }
    /**
     * 
     * @param name the name of the spin motor to remove
     */
    public static void removeSpinMotor(String name) throws Throwable{
        Throwable eThrowable = new Throwable(removingThrowable.getMessage() + checkWhichMethod(3));
        for (int i = 0; i < arrayOfIntakeSpinMotors.size(); i++) {
            if (arrayOfIntakeSpinMotors.get(i).getSpinMotor().getDescription() == name) {
                arrayOfIntakeSpinMotorIds.remove(i);
                arrayOfIntakeSpinMotorNames.remove(i);
                arrayOfIntakeSpinMotors.remove(i);
                break;
            }
        }
        throw eThrowable;
    }
    /**
     * 
     * @param methodNumber our method number assiged to a switch case each with diffrent method names 
     * @return a string for with diffrent method names
     */
    private static String checkWhichMethod(int methodNumber) {
        switch (methodNumber) {
            case 1:
                return "getAPivotMotor()";
                
            case 2:
                return "getASpinMotor()";
                
            case 3:
                return "removeSpinMotor()";
            case 4:
                return "removePivotMotor";
            default:
                return "not found";
        }
    }
    /**
     * 
     * @param spinMotors a array list of spin motors to add to create a new intale
     * @param pivotMotors a array list of pivot motors to add to create a new intake
     */
    public Intake(ArrayList<TalonFX> spinMotors, ArrayList<TalonFX> pivotMotors) {
        for(int i = 0; i <spinMotors.size(); i++){
            arrayOfIntakeSpinMotors.add(new SpinMotor(spinMotors.get(i)));
        }
         for(int i = 0; i <pivotMotors.size(); i++){
            arrayOfPivotMotors.add(new PivotMotor(pivotMotors.get(i)));
        }
    }
    /**
     * A empty intake which passes blank lists into constructor
     */
    public Intake() {
        this(new ArrayList<TalonFX>(), new ArrayList<TalonFX>());
    }

}
