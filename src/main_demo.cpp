#include <pathfinder/pathfinder.h>


int main(int argc, char **argv) {
    auto *pf_ptr = new pathfinder();
    pf_ptr->Config(argc, argv);
    pf_ptr->Start();

}