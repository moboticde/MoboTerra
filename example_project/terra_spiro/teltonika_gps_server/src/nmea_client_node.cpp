#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <chrono>

#include <algorithm>

// Standard C socket headers
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>

// Define constants
constexpr int DEFAULT_PORT = 10001;
constexpr const char* DEFAULT_IP = "127.0.0.1";

// Sample NMEA sentence (GPGGA: GPS Fix Data)
// $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
// 123519: UTC time
// 4807.038,N: Latitude 48 deg 07.038 min North (Simulated: 48.1173)
// 01131.000,E: Longitude 11 deg 31.000 min East (Simulated: 11.5167)
// 1: GPS fix (0=No fix, 1=GPS fix, 2=DGPS fix)
// 08: Satellites used
// 545.4,M: Altitude 545.4 meters
// *47: Checksum
const std::string NMEA_SENTENCE_TEMPLATE = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";


/**
 * @brief Runs the TCP client loop.
 */
void run_tcp_client(const std::string& ip_addr, int port)
{
    std::cout << "Starting TCP client to " << ip_addr << ":" << port << std::endl;
    int sock = 0;
    struct sockaddr_in serv_addr;
    
    // 1. Create socket file descriptor
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "TCP socket creation error." << std::endl;
        return;
    }

    // 2. Setup server address structure
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    
    // Convert IP address from text to binary form
    if (inet_pton(AF_INET, ip_addr.c_str(), &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported." << std::endl;
        close(sock);
        return;
    }

    // 3. Connect to the server
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "Connection Failed. Is the TCP server running?" << std::endl;
        close(sock);
        return;
    }
    std::cout << "Connected to TCP server. Sending NMEA sentences..." << std::endl;

    while (true) 
    {
        // 4. Send the NMEA sentence
        // The NMEA_SENTENCE_TEMPLATE already includes \r\n terminator
        if (send(sock, NMEA_SENTENCE_TEMPLATE.c_str(), NMEA_SENTENCE_TEMPLATE.length(), 0) < 0) {
            std::cerr << "Failed to send data. Server may have closed the connection." << std::endl;
            break;
        }

        // Wait before sending the next sentence
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    close(sock);
}


/**
 * @brief Runs the UDP client loop.
 */
void run_udp_client(const std::string& ip_addr, int port)
{
    std::cout << "Starting UDP client to " << ip_addr << ":" << port << std::endl;
    int sock;
    struct sockaddr_in serv_addr;

    // 1. Create socket file descriptor
    if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "UDP socket creation error." << std::endl;
        return;
    }

    // 2. Setup server address structure
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    
    // Convert IP address from text to binary form
    if (inet_pton(AF_INET, ip_addr.c_str(), &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported." << std::endl;
        close(sock);
        return;
    }

    std::cout << "UDP client ready. Sending NMEA sentences..." << std::endl;

    while (true)
    {
        // 3. Send the NMEA sentence
        // No explicit connection needed for UDP (connectionless)
        if (sendto(sock, NMEA_SENTENCE_TEMPLATE.c_str(), NMEA_SENTENCE_TEMPLATE.length(), 0, 
                   (const struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
            std::cerr << "Failed to send data over UDP." << std::endl;
        }

        // Wait before sending the next sentence
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    close(sock);
}


int main(int argc, char *argv[])
{
    std::string protocol;
    std::string ip_addr = DEFAULT_IP;
    int port = DEFAULT_PORT;

    // --- Command Line Argument Parsing ---
    if (argc < 2 || argc > 4) {
        std::cerr << "Usage: " << argv[0] << " <protocol (tcp|udp)> [IP_ADDRESS] [PORT]" << std::endl;
        std::cerr << "Example: " << argv[0] << " udp 127.0.0.1 10001" << std::endl;
        return 1;
    }

    protocol = argv[1];
    std::transform(protocol.begin(), protocol.end(), protocol.begin(), ::tolower);
    
    if (argc >= 3) {
        ip_addr = argv[2];
    }
    if (argc == 4) {
        try {
            port = std::stoi(argv[3]);
        } catch (const std::exception& e) {
            std::cerr << "Invalid port number provided." << std::endl;
            return 1;
        }
    }
    // -------------------------------------

    if (protocol == "tcp") {
        run_tcp_client(ip_addr, port);
    } else if (protocol == "udp") {
        run_udp_client(ip_addr, port);
    } else {
        std::cerr << "Invalid protocol. Must be 'tcp' or 'udp'." << std::endl;
        return 1;
    }

    return 0;
}