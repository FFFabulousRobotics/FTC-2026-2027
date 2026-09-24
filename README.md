# FTC-2026-2027

Clean FTC project copied from CZL-Mentor---19726mk2. The original project is unchanged.

## TeamCode

Only these Java source files are retained:

- `pedroPathing/Constants.java`
- `pedroPathing/Tuning.java`

Both files are unmodified official Pedro Pathing Quickstart v2.1.2 templates:
https://github.com/Pedro-Pathing/Quickstart/tree/4c556792df0dd72e37575b21bfc8cb77805537af/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing

All previous robot code, automatic paths, AutoPlanner assets, custom tuners, and old build outputs were omitted. FTC controller infrastructure and SDK documentation remain.

The official Constants template intentionally has no robot-specific drivetrain/localizer configuration. Configure the new robot's hardware and tuning values before running Tuning.

## Build

- FTC SDK: 11.1.0 (inherited; the directory name does not upgrade the SDK)
- Pedro Pathing: 2.1.2
- Pedro telemetry: 1.0.0
- Panels: 1.0.12
- Android Gradle Plugin: 8.7.0
- Gradle wrapper: 8.9

Open this directory in Android Studio. `local.properties` points to the existing local Android SDK. Select Android Studio's bundled JDK as the Gradle JDK.

PowerShell build on this computer:

```powershell
$env:JAVA_HOME = 'D:\Android\Android Studio\jbr'
.\gradlew.bat clean :TeamCode:assembleDebug :TeamCode:assembleRelease --console=plain
```

Build verification output is recorded in `build-verification.log`.