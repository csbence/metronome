#ifndef METRONOME_METRONOMEEXCEPTION_HPP
#define METRONOME_METRONOMEEXCEPTION_HPP

#include <exception>
#include <string>
namespace metronome {
class MetronomeException : std::exception {
public:
    MetronomeException(std::string message) : message{std::move(message)} {}

    virtual const char* what() const throw() { return this->message.c_str(); }

protected:
    const std::string message;
};
}

#endif // METRONOME_METRONOMEEXCEPTION_HPP
