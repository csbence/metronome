#include "util/VacuumWorld.hpp" 
#include "catch.hpp" 
#include "easylogging++.h" 

TEST_CASE("VacuumWorld creation", "[VacuumWorld]"){

    VacuumWorld vacuumWorld{0,0,VacuumWorld::State(0,0),VacuumWorld::State(0,0)};

    REQUIRE(vacuumWorld.getWidth() == 0);
    REQUIRE(vacuumWorld.getHeight() == 0);

    
}

TEST_CASE("VacuumWorld::State = operator", "[VacuumWorld]"){

    VacuumWorld::State s = VacuumWorld::State{0,0};
    VacuumWorld::State t = VacuumWorld::State{9,9};

    REQUIRE(s.getX() == 0);
    REQUIRE(s.getY() == 0);
    REQUIRE(t.getX() == 9);
    REQUIRE(t.getY() == 9);

    s = t;

    REQUIRE(s.getX() == 9);
    REQUIRE(s.getY() == 9);

}

TEST_CASE("VacuumWorld setting variables", "[VacuumWorld]"){

    VacuumWorld vacuumWorld{9,9,VacuumWorld::State(2,3),VacuumWorld::State(4,4)};

    vacuumWorld.setWidth(10);
    vacuumWorld.setHeight(13);

    REQUIRE(vacuumWorld.getWidth() == 10);
    REQUIRE(vacuumWorld.getHeight() == 13);

    VacuumWorld::State pair1 = VacuumWorld::State{3,5};
    VacuumWorld::State pair2 = VacuumWorld::State{1,3};
    REQUIRE(vacuumWorld.getNumberBlockedCells() == 0);
    REQUIRE(vacuumWorld.getNumberDirtyCells() == 0);
    REQUIRE(vacuumWorld.getStartLocation().getX() == 2);
    REQUIRE(vacuumWorld.getStartLocation().getY() == 3);

    REQUIRE(pair1.getX() == 3);
    REQUIRE(pair1.getY() == 5);
    REQUIRE(pair2.getX() == 1);
    REQUIRE(pair2.getY()  == 3);

}

TEST_CASE("VacuumWorld getters", "[VacuumWorld]"){
    
    VacuumWorld vacuumWorld{13,9,VacuumWorld::State(3,3),VacuumWorld::State(3,5)};

    vacuumWorld.setWidth(13);
    vacuumWorld.setHeight(9);

    for(int i = 0; i < 10; i++){
        VacuumWorld::State _t = vacuumWorld.randomLocation();

        LOG(INFO) << "R_LOC: " << _t.getX() << " " << _t.getY() << std::endl;
        REQUIRE(vacuumWorld.isLegalLocation(_t));
    }


}

TEST_CASE("VacuumWorld printing", "[VacuumWorld]"){
    VacuumWorld vacuumWorld{3,3,VacuumWorld::State{0,0},VacuumWorld::State{2,2}};


    vacuumWorld.print(LOG(INFO));
    vacuumWorld.addDirtyCell(VacuumWorld::State{1,1});
    vacuumWorld.addDirtyCell(VacuumWorld::State{1,2});

    vacuumWorld.print(LOG(INFO));
    vacuumWorld.addBlockedCell(VacuumWorld::State{2,1});
    vacuumWorld.print(LOG(INFO));

}
