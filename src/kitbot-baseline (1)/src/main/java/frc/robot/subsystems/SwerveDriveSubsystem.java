package frc.robot.subsystems;

import frc.robot.SwerveMod;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveSubsystem extends DriveSubsystem {
    private PoseEstimator s_PoseEstimator = new PoseEstimator();
    public SwerveDriveOdometry swerveOdometry;
    public SwerveMod[] mSwerveMods;
    private Field2d field = new Field2d();
    
    public SwerveDriveSubsystem(PoseEstimator s_PoseEstimator) {
        this.s_PoseEstimator = s_PoseEstimator;

        gyro = new ADXRS450_Gyro();
        gyro.reset();

        //gyro = new PigeonIMU(Constants.Swerve.pigeonID);
        //gyro.configFactoryDefault();
        //gyro.setYaw(0);

        mSwerveMods = new SwerveMod[] {
            new SwerveMod(0, Constants.Swerve.Mod0.constants),
            new SwerveMod(1, Constants.Swerve.Mod1.constants),
            new SwerveMod(2, Constants.Swerve.Mod2.constants),
            new SwerveMod(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(
            Constants.Swerve.swerveKinematics, 
            getGyroYaw(), 
            getModulePositions()
        );
        
        System.out.println(getPose().getX());

        // Set up custom logging to add the current path to a field 2d widget
        SmartDashboard.putData("Field", field);
    }

    @Override
    public Command driveArcade(DoubleSupplier forward, DoubleSupplier rotation) {
        return this.run(() -> {
            double xSpeed = forward.getAsDouble();
            double rot = rotation.getAsDouble();
            drive(new Translation2d(xSpeed, 0.0), rot, false, true);
        });
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = 
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? 
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation, 
                    getHeading()
                ) : 
                new ChassisSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation
                )
            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        
        for(SwerveMod mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    /** Sets the swerve ModuleStates. */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for(SwerveMod mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }

    public void setX(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++){
            states[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(getModuleAngles()[i] + 45));
        }
        setModuleStates(states);
    }

    /** Sets the swerve ModuleStates to zero. */
    public void stop() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++){
            states[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(getModuleAngles()[i]));
        }
        setModuleStates(states);
    }

    public Rotation2d getGyroYaw() {
        return gyro.getRotation2d();
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(0); // ADXRS450 doesn't have pitch
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(0); // ADXRS450 doesn't have roll
    }

    public void zeroGyro(){
        gyro.reset();
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveMod mod : mSwerveMods){
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveMod mod : mSwerveMods){
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void zeroAllEncoders(){
        for(SwerveMod mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public double[] getModuleAngles(){
        double[] angles = new double[4];
        for(int i = 0; i < 4; i++){
            angles[i] = mSwerveMods[i].getCANcoder().getDegrees();
        }
        return angles;
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        s_PoseEstimator.updateSwerve(getGyroYaw(), getModulePositions());
        field.setRobotPose(getPose());
        this.pose = getPose();
    }
}
