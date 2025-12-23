# x_memory2 SIL App

## Prerequisites
1.  **Visual Studio 2022** installed with C++ Desktop Development workload.
2.  **AirSim** installed and built (AirLib and rpclib required).
3.  **AirSim Simulator** (e.g., Blocks or Landscape) running in Unreal Engine.

## How to Build and Run
1.  Open **x64 Native Tools Command Prompt for VS 2022** from the Windows Start Menu.
2.  Navigate to this directory:
    ```cmd
    cd d:\open\airsim\x_memory2
    ```
3.  Run the build script:
    ```cmd
    build_sil.bat
    ```

## Configuration
- The app defaults to **RPC Mode**.
- To switch to **Shared Memory (SHM)** mode, edit `apps/SIL_App.cpp`:
    ```cpp
    // #define USE_RPC 1
    #define USE_SHM 1
    ```

## Troubleshooting
- **'cl' is not recognized**: You are not in the VS Developer Command Prompt.
- **Link errors**: Ensure AirLib and rpclib paths in `build_sil.bat` match your installation.
- **Connection failed**: Ensure AirSim is running and listening on port 41451.
