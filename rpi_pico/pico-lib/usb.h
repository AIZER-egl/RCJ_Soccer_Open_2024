#ifndef PICO_LIB_USB_H
#define PICO_LIB_USB_H

#include <iostream>
#include <string>

#define USB_ENDL "\r\n"

class USB {
public:
    class Printer {
    public:
        template <typename T>
        Printer& operator<<(const T& data) {
            std::cout << data;
            return *this;
        }

        Printer& operator<<(std::ostream& (*manipulator)(std::ostream&)) {
            std::cout << manipulator;
            return *this;
        }
    };

    static Printer print;

    std::string read() {
        std::string data;
        std::cin >> data;
        return data;
    }
};

#endif // PICO_LIB_USB_H
