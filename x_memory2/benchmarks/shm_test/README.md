# SHM Benchmark Package

## Folder Structure
- `src/`: Backup of the source code used for this benchmark.
  - `apps/SIL_App.cpp`: Main application.
  - `core/`, `drivers/`, `modules/`: SIL framework interfaces and implementations.
  - `shm/`: Shared memory header and library.
  - `build.bat`: Standalone build script.
- `Analysis_SHM_Performance.md`: Detailed performance report across 1k, 4k, and 8k frequencies.

## How to Reproduce
1. Open a terminal in the `src/` directory.
2. Run `build.bat` to compile the app.
3. Start AirSim (Ensure `Xsim` model is enabled in `settings.json`).
4. Run the executable with the target frequency:
   ```cmd
   .\build\SIL_App.exe 8000
   ```

## Note for Research/Paper
This data demonstrates that SHM provides a deterministic data path suitable for high-fidelity drone simulation, maintaining <5% jitter at 8kHz sampling rates.
