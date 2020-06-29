
#include <thread>
#include "ThreadManagement/Thread_management.h"
//using namespace cv;
int main(int argc, char *argv[]) {

    auto *management = new ThreadManagement();

    //std::thread image_produce(&ThreadManagement::ImageProduce,management);
    std::thread autoAim(&ThreadManagement::AutoAim,management);
    std::thread bigbuff(&ThreadManagement::Bigbuff,management);
    std::thread comm(&ThreadManagement::Communication_thread,management);
    //image_produce.join();
    autoAim.join();
    bigbuff.join();
    comm.join();
    delete management;
    return 0;
}


