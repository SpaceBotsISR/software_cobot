#include <iostream>

#include "client.hpp"

int main() {
    Client client("127.0.0.1", 8009);

    while (true) {
        std::vector<float> msg = client.receive_msg();
        std::cout << "Received message: ";
        for (auto& i : msg) {
            std::cout << i << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
