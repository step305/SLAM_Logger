#include <iostream>
#include "H5Cpp.h"

int main() {
    H5::H5File  file("log.h5", H5F_ACC_RDONLY);
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
