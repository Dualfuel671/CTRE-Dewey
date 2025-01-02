package frc.robot;


import frc.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.wpilibj.PS4Controller;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


	// The robot's subsystems and commands are defined here...
	public static SwerveDrivetrain m_drivetrain;

	/* Controllers */
	private final PS4Controller driver = new PS4Controller(0);
	private final PS4Controller operator = new PS4Controller(1);
	private final PS4Controller testing = new PS4Controller(2);
	private final PS4Controller limitcontroller = new PS4Controller(3);

	/* Drive Controls */
	private final int translationAxis = PS4Controller.Axis.kLeftY.value;
	private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
	private final int rotationAxis = PS4Controller.Axis.kRightX.value;

	private final JoystickButton isEvading = new JoystickButton(driver, 5);
	private final JoystickButton isRotatingFast = new JoystickButton(driver, 6);
	private final POVButton isLocked = new POVButton(driver, 180);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_drivetrain = new SwerveDrivetrain();
		
		// runs method below to configure button bindings
		configureBindings();

		// add auton options in the constructor as it only happens once
		// m_autoChooser.addOption("none", Autos.none());

		m_drivetrain.setDefaultCommand(
				new SwerveDrive(
						m_drivetrain,
						() -> driver.getRawAxis(translationAxis),
						() -> driver.getRawAxis(strafeAxis),
						() -> driver.getRawAxis(rotationAxis),
						() -> false,
						() -> isEvading.getAsBoolean(),
						() -> isLocked.getAsBoolean(),
						() -> isRotatingFast.getAsBoolean()));
	}

	private void configureBindings() {


		// button to zero gyro -driver select
		 new JoystickButton(driver, 10).onTrue(new
		 InstantCommand(m_drivetrain::zeroGyro));

		

  }

}