
#include <thread>
#include <cstdio>
#include "ThreadPool/Thread_management.h"
//using namespace cv;
int main(int argc, char *argv[]) {

    auto *management = new ThreadManagement();
    auto time0 = static_cast<double>(getTickCount());
    std::thread image_produce(&ThreadManagement::ImageProduce,management);
    std::thread autoAim(&ThreadManagement::AutoAim,management);
    time0 = ((double) getTickCount() - time0) / getTickFrequency();
    cout << "input " << management->inputcounter <<endl;
    cout << "output" << management->outputcounter <<endl;
    std::cout << "use time is " << time0 * 1000 << "ms" << std::endl;
    //std::thread bigbuff(&ThreadManagement::Bigbuff,management);

    image_produce.join();
    autoAim.join();
    //bigbuff.join();
    cout << "input " << management->inputcounter <<endl;
    cout << "output" << management->outputcounter <<endl;
    std::cout << "use time is " << time0 * 1000 << "ms" << std::endl;
    delete management;
    return 0;
}


