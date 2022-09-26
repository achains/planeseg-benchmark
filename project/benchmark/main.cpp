#include <iostream>
#include <string>

#include "peac/peac.hpp"

int main(){
    const std::string iniFileName("configs/peac-office.ini");
    auto algorithm = peac::PEAC(iniFileName);

    std::cout << algorithm.run() << '\n';

    return 0;
}