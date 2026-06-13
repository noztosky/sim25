#include <iostream>
#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: SendCmd <axis: r|p|y|a> <value: deg/m> <duration: sec>\n";
        std::cerr << "Example: SendCmd r 30 1\n";
        std::cerr << "         SendCmd a 5 10\n";
        return 1;
    }

    std::string axis = argv[1];
    std::string value = argv[2];
    std::string duration = argv[3];

    // Basic input validation
    if (axis != "r" && axis != "p" && axis != "y" && axis != "a") {
        std::cerr << "Error: Axis must be 'r' (roll), 'p' (pitch), 'y' (yaw), or 'a' (altitude).\n";
        return 1;
    }

    std::string command = axis + " " + value + " " + duration;

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
