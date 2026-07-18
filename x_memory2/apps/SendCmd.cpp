#include <iostream>
#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

int main(int argc, char* argv[]) {
    std::string command;

    // Word commands (no numeric args): pid = reload params, reset = return to pre-takeoff
    if (argc == 2 && (std::string(argv[1]) == "pid" || std::string(argv[1]) == "reset")) {
        command = argv[1];
    }
    else if (argc >= 4) {
        std::string axis = argv[1];
        std::string value = argv[2];
        std::string duration = argv[3];
        if (axis != "r" && axis != "p" && axis != "y" && axis != "a") {
            std::cerr << "Error: Axis must be 'r' (roll), 'p' (pitch), 'y' (yaw), or 'a' (altitude).\n";
            return 1;
        }
        command = axis + " " + value + " " + duration;
    }
    else {
        std::cerr << "Usage: SendCmd <axis: r|p|y|a> <value: deg/m> <duration: sec>\n";
        std::cerr << "   or: SendCmd pid      (reload PID params from file, live)\n";
        std::cerr << "   or: SendCmd reset    (return to pre-takeoff: reposition + clear state)\n";
        std::cerr << "Example: SendCmd a 5 10\n";
        std::cerr << "         SendCmd r 30 1\n";
        return 1;
    }

    // Initialize Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "Error: Winsock initialization failed.\n";
        return 1;
    }

    // Create socket
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        std::cerr << "Error: Socket creation failed. Error: " << WSAGetLastError() << "\n";
        WSACleanup();
        return 1;
    }

    // Set destination
    sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(5005);
    inet_pton(AF_INET, "127.0.0.1", &dest_addr.sin_addr);

    // Send packet
    int bytes_sent = sendto(sock, command.c_str(), static_cast<int>(command.length()), 0,
                            (SOCKADDR*)&dest_addr, sizeof(dest_addr));

    if (bytes_sent == SOCKET_ERROR) {
        std::cerr << "Error: Failed to send command. Error: " << WSAGetLastError() << "\n";
    } else {
        std::cout << "[SendCmd] Sent command: \"" << command << "\" to 127.0.0.1:5005\n";
    }

    // Clean up
    closesocket(sock);
    WSACleanup();
    return 0;
}
