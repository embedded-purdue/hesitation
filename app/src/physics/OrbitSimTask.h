#pragma once

#include "../BackgroundTask.h"
#include <iostream>

class OrbitSimTask : public BackgroundTask {
  protected:
    void Run() override;
};