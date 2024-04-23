package frc.robot.Subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
      //making intake SparkMax motor controller object for the 775 intake motor
    private CANSparkMax intakeCanSparkMax;

    // constructor for Intake class, this initializes the wrist motor
public Intake(){
    intakeCanSparkMax = new CANSparkMax(15, MotorType.kBrushed);

}
    
/**
 * 
 * The method to run the motor to intake
 */
public void intaking(){
    intakeCanSparkMax.setVoltage(4);
   
}

/**
 * 
 * spin the motor backward to spit out the note...a method to
 */
public void outtaking(){
    intakeCanSparkMax.setVoltage(-1);
}
/**
 * 
 * Method for stopping intake
 */
public void stopIntake(){
    intakeCanSparkMax.stopMotor();
}












}
