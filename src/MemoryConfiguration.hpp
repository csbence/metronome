#ifndef METRONOME_MEMORYCONFIGURATION_HPP
#define METRONOME_MEMORYCONFIGURATION_HPP

namespace metronome {

class Memory {
public:
    static constexpr int OPEN_LIST_SIZE{30000000};
    static constexpr int NODE_LIMIT{10};
};
}

#endif //METRONOME_MEMORYCONFIGURATION_HPP
