#include <iostream>

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./HoleDetection <file-name>";
        return 0;
    }

    std::string data(argv[1]);

    std::cout << "Hello World\n";
    return 0;
}
