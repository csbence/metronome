#ifndef VACUUM_WORLD_HPP
#define VACUUM_WORLD_HPP

#include <boost/assert.hpp>
#include <cstdlib>
#include <vector>
#include <functional>

class VacuumWorld{
    private:
        int width = 0;
        int height = 0;
        std::vector<std::pair<int,int>> blockedCells;
        std::vector<std::pair<int,int>> dirtyCells;
        std::pair<int, int> startLocation;
        std::pair<int ,int> goalLocation;
        int initialAmountDirty = 1;
    public:
        std::pair<int ,int> randomLocation(){
            int _first = rand() % width;
            int _second = rand() % height;

            std::pair<int, int> ret;
            ret.first = _first;
            ret.second = _second;
            return ret;
        }
        std::pair<int, int> getGoal(){
            return goalLocation;
        }
        bool isGoal(std::pair<int, int> location){
            return location.first == goalLocation.first &&
                   location.second == goalLocation.second;
        }
        bool inBlockedCells(std::pair<int,int> location){
            for (auto it : blockedCells){
                if(it.first == location.first && it.second == location.second){
                    return true;
                }
            }
            return false;
        }
        bool isLegalLocation(std::pair<int,int> location){
            return location.first >= 0 && location.second >= 0 && location.first < width
                    && location.second < height && !inBlockedCells(location);
        }
        void setWidth(const int newWidth){
            width = newWidth;
        }
        void setHeight(const int newHeight){
            height = newHeight;
        }
        int getWidth(){
            return width;
        }
        int getHeight(){
            return height;
        }
        bool addBlockedCell(const std::pair<int,int> toAdd){
            if(isLegalLocation(toAdd)){
                blockedCells.push_back(toAdd);
                return true;
            }
            return false;
        }
        bool addDirtyCell(const std::pair<int,int> toAdd){
            if(isLegalLocation(toAdd)){
                dirtyCells.push_back(toAdd);
                return true;
            }
            return false;
        }
        bool changeStartLocation(const std::pair<int,int> location){
            if(isLegalLocation(location)){
                startLocation = location;
                return true;
            }
            return false;
        }
        std::vector<std::pair<int,int>>::size_type getNumberBlockedCells(){
            return blockedCells.size();
        }
        std::vector<std::pair<int,int>>::size_type getNumberDirtyCells(){
            return dirtyCells.size();
        }
        std::pair<int,int> getStartLocation(){
            return startLocation;
        }
};
#endif
