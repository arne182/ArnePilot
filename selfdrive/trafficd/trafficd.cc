#include "traffic.h"

using namespace std;

zdl::DlSystem::Runtime_t checkRuntime() {
    static zdl::DlSystem::Version_t Version = zdl::SNPE::SNPEFactory::getLibraryVersion();
    static zdl::DlSystem::Runtime_t Runtime;
    std::cout << "SNPE Version: " << Version.asString().c_str() << std::endl; //Print Version number
    if (zdl::SNPE::SNPEFactory::isRuntimeAvailable(zdl::DlSystem::Runtime_t::GPU)) {
        Runtime = zdl::DlSystem::Runtime_t::GPU;
    } else {
        Runtime = zdl::DlSystem::Runtime_t::CPU;
    }
    return Runtime;
}

void initModel() {
    zdl::DlSystem::Runtime_t runt=checkRuntime();
    initializeSNPE(runt);
}

int main(){
    //usleep(5000000);
    //set_realtime_priority(2);
    initModel(); // init model
    printf("here!\n");
}
