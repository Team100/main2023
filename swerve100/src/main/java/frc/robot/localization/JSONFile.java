package frc.robot.localization;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;

public class JSONFile {
    public static Path getPath() {
        return Filesystem.getDeployDirectory().toPath().resolve("2023-chargedup.json");
    }
}
