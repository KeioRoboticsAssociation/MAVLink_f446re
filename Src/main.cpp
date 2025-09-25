#include "main.hpp"

// Forward declarations for C interface
extern "C" {
    void cpp_setup();
    void cpp_loop();
}


void setup() {
    // Initialize the new SystemContext architecture
    cpp_setup();
}

void loop() {
    // Update the SystemContext architecture
    cpp_loop();
}

