#pragma once

#include <thread>
#include <atomic>
#include <chrono>

class BackgroundTask {
  public:
    virtual ~BackgroundTask() {
        Stop();
    }

    void Start() {
        if (running)
            return;

        running      = true;
        workerThread = std::thread(&BackgroundTask::RunWrapper, this);
    }

    void Stop() {
        running = false;

        if (workerThread.joinable())
            workerThread.join();
    }

  protected:
    std::atomic<bool> running{false};

    virtual void Run() = 0;

  private:
    std::thread workerThread;

    void RunWrapper() {
        Run();
    }
};