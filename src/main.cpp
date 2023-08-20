#include "MyApplication.h"

int main(void)
{
    MyApplication* myApp = new MyApplication;
    myApp->run();
    delete myApp;
}