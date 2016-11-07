#ifndef METRONOME_STRING_HPP
#define METRONOME_STRING_HPP

#include <string>
#include <vector>
#include <sstream>

void split(const std::string &string, char delimiter, std::vector<std::string> &tokens) {
    std::stringstream ss;
    ss.str(string);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        tokens.push_back(item);
    }
}

std::vector<std::string> split(const std::string &string, char delimiter) {
    std::vector<std::string> tokens;
    split(string, delimiter, tokens);
    return tokens;
}

#endif //METRONOME_STRING_HPP
