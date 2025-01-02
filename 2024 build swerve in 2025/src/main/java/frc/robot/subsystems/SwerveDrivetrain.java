package frc.robot.subsystems;

import frc.lib.geometry.Translation2dPlus;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Arrays;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.proto.Photon;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class SwerveDrivetrain extends SubsystemBase {
    private static final Translation2d[] WHEEL_POSITIONS = Arrays.copyOf(Constants.moduleTranslations,
            Constants.moduleTranslations.length);;
    public boolean teleop = false;
    public SwerveModule[] m_swerveMods;
    public Pigeon2 m_gyro;
    public PhotonVision m_photonVision;
    private final SwerveDrivePoseEstimator poseEstimator;

    public SwerveDrivetrain() {
        m_gyro = new Pigeon2(32, "DriveBus");
        m_photonVision = new PhotonVision();
        m_gyro.reset();

        m_swerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Mod0.constants),
                new SwerveModule(1, Constants.Mod1.constants),
                new SwerveModule(2, Constants.Mod2.constants),
                new SwerveModule(3, Constants.Mod3.constants)
        };

        poseEstimator = new SwerveDrivePoseEstimator(Constants.swerveKinematics,
                getYaw(), getModulePositions(),
                new Pose2d(0, 0, m_gyro.getRotation2d()), Constants.STATE_STDS, Constants.VISION_STDS);


    }

    /**
     * Drive method for swerve drivetrain.
     * 
     * @param translation   The desired translation vector.
     * @param rotation      The desired rotation value.
     * @param fieldRelative Whether the translation is field relative.
     * @param isOpenLoop    Whether the drive is open loop.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop,
            boolean isEvading, boolean isLocked) {

        if (isLocked) {

            final SwerveModuleState[] swerveModuleStates = new SwerveModuleState[] {
                    new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
                    new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
                    new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
            };

            for (SwerveModule mod : m_swerveMods) {
                mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
            }

        } else {

            final Translation2d centerOfRotation;

            if (isEvading && fieldRelative) {
                centerOfRotation = getCenterOfRotation(translation.getAngle(), rotation);
            } else {
                centerOfRotation = new Translation2d();
            }

            final ChassisSpeeds chassisSpeeds;

            if (fieldRelative) {

                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getPose().getRotation());

            } else {

                chassisSpeeds = new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation);
            }

            final var swerveModuleStates = Constants.swerveKinematics.toSwerveModuleStates(chassisSpeeds,
                    centerOfRotation);
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED);

            for (SwerveModule mod : m_swerveMods) {
                mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
            }

        }

    }

    private Translation2d getCenterOfRotation(final Rotation2d direction, final double rotation) {
        final var here = new Translation2dPlus(1.0, direction.minus(getYaw()));

        var cwCenter = WHEEL_POSITIONS[0];
        var ccwCenter = WHEEL_POSITIONS[WHEEL_POSITIONS.length - 1];

        for (int i = 0; i < WHEEL_POSITIONS.length - 1; i++) {
            final var cw = WHEEL_POSITIONS[i];
            final var ccw = WHEEL_POSITIONS[i + 1];

            if (here.isWithinAngle(cw, ccw)) {
                cwCenter = ccw;
                ccwCenter = cw;
            }
        }

        // if clockwise
        if (Math.signum(rotation) == 1.0) {
            return cwCenter;
        } else if (Math.signum(rotation) == -1.0) {
            return ccwCenter;
        } else {
            return new Translation2d();
        }
    }

    /**
     * Set chassis speeds for the swerve drivetrain.
     * 
     * @param targetSpeeds The target chassis speeds.
     */
    public void setChassisSpeeds(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = Constants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Set desired states for each swerve module.
     * 
     * @param desiredStates The desired states for each swerve module.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED);

        for (SwerveModule mod : m_swerveMods) {
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
        }
    }

    /**
     * Gets the current robot pose in the field coordinate system.
     * 
     * @return The current robot pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to a specified pose.
     * 
     * @param pose The desired pose to reset odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        System.out.println(pose.getX());
        System.out.println(pose.getY());
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_swerveMods) {
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_swerveMods) {
            positions[mod.m_moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Zeros the gyro by resetting its accumulated yaw.
     */
    public void zeroGyro() {
        m_gyro.reset();
        resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), Rotation2d.fromDegrees(0)));
    }

    /**
     * Gets the current yaw (rotation around Z-axis) from the gyro.
     * 
     * @return The current yaw as a Rotation2d.
     */
    public Rotation2d getYaw() {
        return m_gyro.getRotation2d();

    }

   
   


 


 

   

    @Override
    public void periodic() {
        poseEstimator.update(getYaw(), getModulePositions());
        
        
           var pose = m_photonVision.getRobotPose();
           pose.ifPresent(
            est -> {
            poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
           });

        for (SwerveModule mod : m_swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Angle",
                    mod.getCANcoder().getDegrees() - mod.m_angleOffset.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Integrated",
                    mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity",
                    mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("real robot pose x", getPose().getX());
        SmartDashboard.putNumber("real robot pose y", getPose().getY());
        SmartDashboard.putNumber("real robot pose rot",
                getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro Rot", getYaw().getDegrees());


    }
}