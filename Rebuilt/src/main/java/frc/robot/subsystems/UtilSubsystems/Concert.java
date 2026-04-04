package frc.robot.subsystems.UtilSubsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class Concert extends SubsystemBase {
    

    private static Concert INSTANCE;

    public static Concert getInstance() {
        if (INSTANCE == null) INSTANCE = new Concert();
        return INSTANCE;
    }

    private static void Initialize() {
        INSTANCE.addAllInstruments();
        INSTANCE.loadInInstruments();
        INSTANCE.changeMelody("EXAMPLE.chrp");
    }

    private ArrayList<TalonFX> instrumentList = new ArrayList<>();
    private Orchestra concertHall = new Orchestra();

    private void addAllInstruments() {
        addInstrument(ClimberSubsystem.getInstance().getMotors());
        addInstrument(RobotContainer.drivetrain.getMotors());
        addInstrument(IntakeSubsystem.getInstance().getMotors());
        addInstrument(ShooterSubsystem.getInstance().getMotors());
        addInstrument(TransferSubsystem.getInstance().getMotor());
        addInstrument(TurretSubsystem.getInstance().getMotor());
    }

    private void addInstrument(TalonFX instrument) {
        instrumentList.add(instrument);
    }

    private void addInstrument(ArrayList<TalonFX> instruments) {
        for (TalonFX instrument : instruments) {
            addInstrument(instrument);
        }
    }

    private void loadInInstruments() {
        for (TalonFX instrument : instrumentList) {
            concertHall.addInstrument(instrument);
        }
    }

    private void playMelody() {
        concertHall.play();
    }

    private void pauseMelody() {
        concertHall.pause();
    }

    private void stopMelody() {
        concertHall.stop();
    }

    private void changeMelody(String filePath) {
        concertHall.loadMusic(filePath);
    }
}
