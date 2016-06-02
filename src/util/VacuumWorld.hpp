#ifndef VACUUM_WORLD_HPP
#define VACUUM_WORLD_HPP

#include <boost/assert.hpp>
#include <cstdlib>
#include <functional>
#include <vector>

class VacuumWorld {

public:
  /*
   State <- location of the agent as a pair
   Action <- value representing the action taken {N,S,E,W,V} = {0,1,2,3,4}
   Cost <- value for taking action from a state
  */
  class State {
  private:
    unsigned int x;
    unsigned int y;
    bool dirty;
    bool object;
    void swap(State &first, State &second) {
      using std::swap;
      swap(first.x, second.x);
      swap(first.y, second.y);
    }

  public:
    State() : x(0), y(0), dirty(false), object(false) {}
    State(unsigned int x, unsigned int y)
        : x(x), y(y), dirty(false), object(false) {}
    State &operator=(State toCopy) {
      swap(*this, toCopy);
      return *this;
    }
    void setDirty() { dirty = !dirty; }
    void setObject() { object = !object; }
    bool isDirty() const { return dirty; }
    bool isObject() const { return object; }
    unsigned int getX() const { return x; }
    unsigned int getY() const { return y; }
  };
  class Action {
  private:
    unsigned int value;
    char rep;
    char actions[5] = {'N', 'E', 'S', 'W', 'V'};

  public:
    Action(unsigned int v) : value(v), rep(actions[v]) {}
    char getRepresentation() const { return rep; }
    unsigned int getValue() const { return value; }
  };
  typedef unsigned long Cost;

private:
  /*
   maxActions <- maximum number of actions
   width/height <- internal size representation of world
   blockedCells <- stores locations of the objects in world
   dirtyCells <- stores locations of dirt in the world
   startLocation <- where the agent begins
   goalLocation <- where the agent needs to end up
   initalAmountDirty <- how many cells are dirty
   initialCost <- constant cost value
  */
  const unsigned int maxActions = 5;
  unsigned int width = 0;
  unsigned int height = 0;
  std::vector<State> blockedCells;
  std::vector<State> dirtyCells;
  State startLocation{0, 0};
  State goalLocation{0, 0};
  unsigned int initialAmountDirty = 1;
  const unsigned long initialCost = 1.0;
  State transitionLegal(State s, State t){
    if(isLegalLocation(t)){
      return t;
    }
    return s;
  }

public:
  VacuumWorld(unsigned int width, unsigned int height, State start, State goal)
      : width(width), height(height), startLocation(start), goalLocation(goal) {
  }
  State transition(State s, Action a){
    if(a.getRepresentation() == 'N'){
      return transitionLegal(s,State{s.getX(),(s.getY()+1) % height});
    }
    else if(a.getRepresentation() == 'E'){
      return transitionLegal(s,State{(s.getX()+1) % width, s.getY()});
    }
    else if(a.getRepresentation() == 'S'){
      return transitionLegal(s,State{s.getX(), (s.getY()-1) % height});
    }
    else if(a.getRepresentation() == 'W'){
      return transitionLegal(s,State{(s.getX()-1) % width, s.getY()});
    }
    else{
      if(s.isDirty()){
        s.setDirty();
      }
      return s;
    }
  }
  void print(std::ostream &out) {
    char grid[height][width];
    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        grid[i][j] = '_';
      }
    }
    for (auto it : blockedCells) {
      grid[(it).getX()][(it).getY()] = '#';
    }
    for (auto it : dirtyCells) {
      grid[(it).getX()][(it).getY()] = '$';
    }
    grid[startLocation.getX()][startLocation.getY()] = '@';
    grid[goalLocation.getX()][goalLocation.getY()] = 'G';

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        out << grid[i][j];
      }
      out << std::endl;
    }
    out << std::endl << std::endl;
  }
  /***
   *** Given a state and action pair give the cost
   *** for taking the action in the state
   *** TODO: make it take a cost function instead of constant
   ***/
  Cost getCost(State s, Action a) const { return initialCost; }
  Action randomAction() const {
    return Action{rand() % maxActions};
  }
  State randomLocation() const {
    unsigned int x = rand() % width;
    unsigned int y = rand() % height;

    return State{x, y};
  }
  State getGoal() const {
    return State{goalLocation.getX(), goalLocation.getY()};
  }
  bool isGoal(const State location) const {
    return location.getX() == goalLocation.getX() &&
           location.getY() == goalLocation.getY();
  }
  bool inBlockedCells(const State location) const {
    return location.isObject();
  }
  bool inDirtyCells(const State location) const { return location.isDirty(); }
  bool isLegalLocation(State location) const {
    return location.getX() < width && location.getY() < height &&
           !inBlockedCells(location);
  }
  void setWidth(const int newWidth) { width = newWidth; }
  void setHeight(const int newHeight) { height = newHeight; }
  int getWidth() const { return width; }
  int getHeight() const { return height; }
  bool addBlockedCell(const State toAdd) {
    if (isLegalLocation(toAdd)) {
      toAdd.isObject();
      blockedCells.push_back(toAdd);
      return true;
    }
    return false;
  }
  bool addDirtyCell(const State toAdd) {
    if (isLegalLocation(toAdd)) {
      toAdd.isDirty();
      dirtyCells.push_back(toAdd);
      return true;
    }
    return false;
  }
  std::vector<State>::size_type getNumberBlockedCells() const {
    return blockedCells.size();
  }
  std::vector<State>::size_type getNumberDirtyCells() const {
    return dirtyCells.size();
  }
  State getStartLocation() const { return startLocation; }
};
#endif
