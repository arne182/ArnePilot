#include <SNPE/SNPE.hpp>
#include <SNPE/SNPEBuilder.hpp>
#include <SNPE/SNPEFactory.hpp>
#include <DlContainer/IDlContainer.hpp>
#include <DlSystem/DlError.hpp>
#include <DlSystem/ITensor.hpp>
#include <DlSystem/ITensorFactory.hpp>
#include <DlSystem/IUserBuffer.hpp>
#include <DlSystem/IUserBufferFactory.hpp>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <array>

#include <stdio.h>
#include <stdlib.h>

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <unistd.h>

#include "common/visionbuf.h"
#include "common/visionipc.h"
#include "common/timing.h"
#include "messaging.hpp"
#include "cereal/gen/cpp/arne182.capnp.h"

#include <capnp/message.h>
#include <capnp/serialize-packed.h>

int main();
