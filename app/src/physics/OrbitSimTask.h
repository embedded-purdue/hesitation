#pragma once

#include "../BackgroundTask.h"

class OrbitSimTask : public BackgroundTask {
  public:
    bool Initialize();

  protected:
    void Run() override;
};