#include "traffic.h"

//#include <sched.h>

using namespace std;

std::unique_ptr<zdl::SNPE::SNPE> snpe;
volatile sig_atomic_t do_exit = 0;

const std::vector<std::string> modelLabels = {"SLOW", "GREEN", "NONE"};
const int numLabels = modelLabels.size();
const double modelRate = 1 / 5.;  // 5 Hz

const int original_shape[3] = {874, 1164, 3};   // global constants
const int original_size = 874 * 1164 * 3;
const int cropped_shape[3] = {665, 814, 3};
const int cropped_size = 665 * 814 * 3;

const int horizontal_crop = 175;
const int top_crop = 0;
const int hood_crop = 209;
const double msToSec = 1 / 1000.;  // multiply
const double secToUs = 1e+6;


int main(){

}
