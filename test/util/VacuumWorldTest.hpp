#include "util/VacuumWorld.hpp" 
#include "catch.hpp" 
#include "easylogging++.h" 

TEST_CASE("VacuumWorld creation", "[VacuumWorld]"){

    VacuumWorld vacuumWorld; 

    REQUIRE(vacuumWorld.getWidth() == 0);
    REQUIRE(vacuumWorld.getHeight() == 0);

    
}

TEST_CASE("VacuumWorld setting variables", "[VacuumWorld]"){

    VacuumWorld vacuumWorld;

    vacuumWorld.setWidth(10);
    vacuumWorld.setHeight(13);
    REQUIRE(vacuumWorld.getWidth() == 10);
    REQUIRE(vacuumWorld.getHeight() == 13);

    std::pair<int,int> pair1 = std::make_pair(3,5);
    std::pair<int,int> pair2 = std::make_pair(1,3);
    REQUIRE(vacuumWorld.getNumberBlockedCells() == 0);
    REQUIRE(vacuumWorld.getNumberDirtyCells() == 0);
    REQUIRE(vacuumWorld.getStartLocation().first == 0);
    REQUIRE(vacuumWorld.getStartLocation().second == 0);

    REQUIRE(vacuumWorld.changeStartLocation(pair1)); 
    REQUIRE(vacuumWorld.getStartLocation().first == 3); 
    REQUIRE(vacuumWorld.getStartLocation().second == 5);

    REQUIRE(vacuumWorld.changeStartLocation(pair2));
    REQUIRE(vacuumWorld.getStartLocation().first == 1);
    REQUIRE(vacuumWorld.getStartLocation().second == 3);

}

TEST_CASE("VacuumWorld getters", "[VacuumWorld]"){
    
    VacuumWorld vacuumWorld;

    vacuumWorld.setWidth(13);
    vacuumWorld.setHeight(9);

    for(int i = 0; i < 10; i++){
        std::pair<int, int> _t = vacuumWorld.randomLocation();
        std::cout << "R_LOC: " << _t.first << " " << _t.second << std::endl;
        REQUIRE(vacuumWorld.isLegalLocation(_t));
    }

}
