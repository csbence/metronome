#ifndef METRONOME_SUCCESSORBUNDLE_HPP
#define METRONOME_SUCCESSORBUNDLE_HPP
template <typename Domain>
class SuccessorBundle {
public:

    typedef typename Domain::State State;
    typedef typename Domain::Action Action;
    typedef typename Domain::Cost Cost;

    SuccessorBundle(State state, Action action, Cost actionCost)
            : state(state), action(action), actionCost(actionCost) {
    }

    const State state;
    const Action action;
    const Cost actionCost;
};
#endif // METRONOME_SUCCESSORBUNDLE_HPP
