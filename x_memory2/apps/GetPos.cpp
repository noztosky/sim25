// Minimal RPC query tool: prints the multirotor NED position (x=North, y=East, z=Down).
// Used to verify left/right (Y) movement while testing flight commands.
#include "../drivers/rpc/RPC_Driver.hpp"
#include <cstdio>

int main() {
    RPC_Driver rpc;
    if (!rpc.connect()) {
        std::fprintf(stderr, "GetPos: failed to connect to AirSim RPC (41451)\n");
        return 1;
    }
    try {
        auto state = rpc.get_client()->getMultirotorState();
        auto p = state.getPosition();   // NED, meters
        auto v = state.kinematics_estimated.twist.linear; // NED velocity
        // North(X), East(Y, +=right), Down(Z); Up = -Z
        std::printf("POS x(N)=%.3f y(E)=%.3f z(D)=%.3f up=%.3f | vel x=%.3f y=%.3f z=%.3f\n",
                    p.x(), p.y(), p.z(), -p.z(), v.x(), v.y(), v.z());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "GetPos: RPC query failed: %s\n", e.what());
        rpc.close();
        return 1;
    }
    rpc.close();
    return 0;
}
