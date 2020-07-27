#pragma clang diagnostic ignored "-Wexceptions"
#include "traffic.h"

//#include <sched.h>

using namespace std;

std::unique_ptr<zdl::SNPE::SNPE> snpe;
volatile sig_atomic_t do_exit = 0;

const std::vector<std::string> modelLabels = {"SLOW", "GREEN", "NONE"};
const int numLabels = modelLabels.size();
const double modelRate = 1 / 5.;  // 5 Hz
const bool debug_mode = true;

const int original_shape[3] = {874, 1164, 3};   // global constants
const int original_size = 874 * 1164 * 3;
const int cropped_shape[3] = {665, 814, 3};
const int cropped_size = 665 * 814 * 3;

const int horizontal_crop = 175;
const int top_crop = 0;
const int hood_crop = 209;
const double msToSec = 1 / 1000.;  // multiply
const double secToUs = 1e+6;


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

void initializeSNPE(zdl::DlSystem::Runtime_t runtime) {
    std::unique_ptr<zdl::DlContainer::IDlContainer> container;
    container = zdl::DlContainer::IDlContainer::open("../../models/traffic_model.dlc");
    zdl::SNPE::SNPEBuilder snpeBuilder(container.get());
    snpe = snpeBuilder.setOutputLayers({})
                      .setRuntimeProcessor(runtime)
                      .setUseUserSuppliedBuffers(false)
                      .setPerformanceProfile(zdl::DlSystem::PerformanceProfile_t::HIGH_PERFORMANCE)
                      .setCPUFallbackMode(true)
                      .build();
}

std::unique_ptr<zdl::DlSystem::ITensor> loadInputTensor(std::unique_ptr<zdl::SNPE::SNPE> &snpe, std::vector<float> inputVec) {
    std::unique_ptr<zdl::DlSystem::ITensor> input;
    const auto &strList_opt = snpe->getInputTensorNames();

    if (!strList_opt) throw std::runtime_error("Error obtaining Input tensor names");
    const auto &strList = *strList_opt;
    assert (strList.size() == 1);

    const auto &inputDims_opt = snpe->getInputDimensions(strList.at(0));
    const auto &inputShape = *inputDims_opt;

    input = zdl::SNPE::SNPEFactory::getTensorFactory().createTensor(inputShape);

    /* Copy the loaded input file contents into the networks input tensor. SNPE's ITensor supports C++ STL functions like std::copy() */
    std::copy(inputVec.begin(), inputVec.end(), input->begin());
    return input;
}

zdl::DlSystem::ITensor* executeNetwork(std::unique_ptr<zdl::SNPE::SNPE>& snpe, std::unique_ptr<zdl::DlSystem::ITensor>& input) {
    static zdl::DlSystem::TensorMap outputTensorMap;
    snpe->execute(input.get(), outputTensorMap);
    zdl::DlSystem::StringList tensorNames = outputTensorMap.getTensorNames();

    const char* name = tensorNames.at(0);  // only should the first
    auto tensorPtr = outputTensorMap.getTensor(name);
    return tensorPtr;
}

//int set_realtime_priority(int level) {
//#ifdef __linux__
//
//  long tid = syscall(SYS_gettid);

//  // should match python using chrt
//  struct sched_param sa;
//  memset(&sa, 0, sizeof(sa));
//  sa.sched_priority = level;
//  return sched_setscheduler(tid, SCHED_FIFO, &sa);
//#else
//  return -1;
//#endif
//}

void initModel() {
    zdl::DlSystem::Runtime_t runt=checkRuntime();
    initializeSNPE(runt);
}

void sendPrediction(std::vector<float> modelOutputVec, PubSocket* traffic_lights_sock) {
    float modelOutput[numLabels];
    for (int i = 0; i < numLabels; i++){  // convert vector to array for capnp
        modelOutput[i] = modelOutputVec[i];
    }

    kj::ArrayPtr<const float> modelOutput_vs(&modelOutput[0], numLabels);
    capnp::MallocMessageBuilder msg;
    cereal::EventArne182::Builder event = msg.initRoot<cereal::EventArne182>();
    event.setLogMonoTime(nanos_since_boot());

    auto traffic_lights = event.initTrafficModelRaw();
    traffic_lights.setPrediction(modelOutput_vs);

    auto words = capnp::messageToFlatArray(msg);
    auto bytes = words.asBytes();
    traffic_lights_sock->send((char*)bytes.begin(), bytes.size());
}

std::vector<float> runModel(std::vector<float> inputVector) {
    std::unique_ptr<zdl::DlSystem::ITensor> inputTensor = loadInputTensor(snpe, inputVector);  // inputVec)
    zdl::DlSystem::ITensor* tensor = executeNetwork(snpe, inputTensor);

    std::vector<float> outputVector;
    for (auto it = tensor->cbegin(); it != tensor->cend(); ++it ){
        float op = *it;
        outputVector.push_back(op);
    }
    return outputVector;
}

void sleepFor(double sec) {
    usleep(sec * secToUs);
}

double rateKeeper(double loopTime, double lastLoop) {
    double toSleep;
    if (lastLoop < 0){  // don't sleep if last loop lagged
        lastLoop = std::max(lastLoop, -modelRate);  // this should ensure we don't keep adding negative time to lastLoop if a frame lags pretty badly
                                                    // negative time being time to subtract from sleep time
        // std::cout << "Last frame lagged by " << -lastLoop << " seconds. Sleeping for " << modelRate - (loopTime * msToSec) + lastLoop << " seconds" << std::endl;
        toSleep = modelRate - (loopTime * msToSec) + lastLoop;  // keep time as close as possible to our rate, this reduces the time slept this iter
    } else {
        toSleep = modelRate - (loopTime * msToSec);
    }
    if (toSleep > 0){  // don't sleep for negative time, in case loop takes too long one iteration
        sleepFor(toSleep);
    } else {
        std::cout << "trafficd lagging by " << -(toSleep / msToSec) << " ms." << std::endl;
    }
    return toSleep;
}

void set_do_exit(int sig) {
    std::cout << "trafficd - received signal: " << sig << std::endl;
    std::cout << "trafficd - shutting down!" << std::endl;
    do_exit = 1;
}

uint8_t clamp(int16_t value) {
    return value<0 ? 0 : (value>255 ? 255 : value);
}

static std::vector<float> getFlatVector(const VIPCBuf* buf, const bool returnBGR) {
    // returns RGB if returnBGR is false
    const size_t width = original_shape[1];
    const size_t height = original_shape[0];

    uint8_t *y = (uint8_t*)buf->addr;
    uint8_t *u = y + (width * height);
    uint8_t *v = u + (width / 2) * (height / 2);

    int b, g, r;
    std::vector<float> bgrVec;
    for (int y_cord = top_crop; y_cord < (original_shape[0] - hood_crop); y_cord++) {
        for (int x_cord = horizontal_crop; x_cord < (original_shape[1] - horizontal_crop); x_cord++) {
            int yy = y[(y_cord * width) + x_cord];
            int uu = u[((y_cord / 2) * (width / 2)) + (x_cord / 2)];
            int vv = v[((y_cord / 2) * (width / 2)) + (x_cord / 2)];

            r = 1.164 * (yy - 16) + 1.596 * (vv - 128);
            g = 1.164 * (yy - 16) - 0.813 * (vv - 128) - 0.391 * (uu - 128);
            b = 1.164 * (yy - 16) + 2.018 * (uu - 128);

            if (returnBGR){
                bgrVec.push_back(clamp(b) / 255.0);
                bgrVec.push_back(clamp(g) / 255.0);
                bgrVec.push_back(clamp(r) / 255.0);
            } else {
                bgrVec.push_back(clamp(r) / 255.0);
                bgrVec.push_back(clamp(g) / 255.0);
                bgrVec.push_back(clamp(b) / 255.0);
            }
        }
    }
    return bgrVec;
}


int main(){
    signal(SIGINT, (sighandler_t)set_do_exit);
    signal(SIGTERM, (sighandler_t)set_do_exit);
    int err;
    //usleep(5000000);
    //set_realtime_priority(2);
    initModel(); // init model

    VisionStream stream;

    Context* c = Context::create();
    PubSocket* traffic_lights_sock = PubSocket::create(c, "trafficModelRaw");
    assert(traffic_lights_sock != NULL);
    while (!do_exit){  // keep traffic running in case we can't get a frame (mimicking modeld)
        VisionStreamBufs buf_info;
        err = visionstream_init(&stream, VISION_STREAM_YUV, true, &buf_info);
        if (err != 0) {
            printf("trafficd: visionstream fail\n");
            usleep(100000);
            continue;
        }

        double loopStart;
        double lastLoop = 0;
        while (!do_exit){
            loopStart = millis_since_boot();

            VIPCBuf* buf;
            VIPCBufExtra extra;
            buf = visionstream_get(&stream, &extra);
            if (buf == NULL) {
                printf("trafficd: visionstream get failed\n");
                break;
            }

            std::vector<float> imageVector = getFlatVector(buf, true);  // writes float vector to inputVector
            std::vector<float> modelOutputVec = runModel(imageVector);

            sendPrediction(modelOutputVec, traffic_lights_sock);

            if (debug_mode) {
                int predictionIndex = std::max_element(modelOutputVec.begin(), modelOutputVec.end()) - modelOutputVec.begin();
//                std::cout << "Model prediction: " << modelLabels[predictionIndex] << " (";
                printf("Model prediction: %d (%s%%)\n", predictionIndex, modelLabels[predictionIndex].c_str());
                std::cout << "Current frequency: " << 1 / ((millis_since_boot() - loopStart) * msToSec) << " Hz" << std::endl;
            } else {  // don't run ratekeeper while debugging
                lastLoop = rateKeeper(millis_since_boot() - loopStart, lastLoop);
            }
        }
    }
    std::cout << "trafficd is dead" << std::endl;
    visionstream_destroy(&stream);
    return 0;
}
