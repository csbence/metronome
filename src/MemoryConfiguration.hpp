#ifndef METRONOME_MEMORYCONFIGURATION_HPP
#define METRONOME_MEMORYCONFIGURATION_HPP

namespace metronome {

class Memory {
public:
    static constexpr std::size_t OPEN_LIST_SIZE{30000000};
    static constexpr std::size_t NODE_LIMIT{30000000};
};
}

#endif //METRONOME_MEMORYCONFIGURATION_HPP
