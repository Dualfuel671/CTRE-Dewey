package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    // making motor object
private TalonFX shootermotor1Fx, shootermotor2Fx;


    // constructor for shooter class, this initializes the wrist motor
public Shooter(){
    shootermotor1Fx = new TalonFX(13);
    shootermotor2Fx = new TalonFX(14);
    };
    /**
     * 
     * TalonFX method to turn shooter motor counterclockwise(maybe or maybe clockwise) at 1 volt 
     */

    public void spinShooter(){
        shootermotor1Fx.setVoltage(6);
        shootermotor2Fx.setVoltage(6);
    };

    /**
     * 
     * method to stop talonFX shooter motor
     */

    public void stopShooter(){
        shootermotor1Fx.stopMotor();
        shootermotor2Fx.stopMotor();
    };
    




























}
