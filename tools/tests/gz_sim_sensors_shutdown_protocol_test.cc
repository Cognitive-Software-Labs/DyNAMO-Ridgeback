#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

// Supplemental protocol harness for the synchronization fields and predicates
// in gz-sim8_8.15.0 src/systems/sensors/Sensors.cc. It deliberately avoids
// constructing Gazebo or a rendering engine.
class SensorsShutdownProtocol
{
  public: std::atomic<bool> running{true};
  public: std::atomic<bool> updateAvailable{true};
  public: std::chrono::steady_clock::duration updateTimeApplied{1ns};
  public: std::chrono::steady_clock::duration updateTimeToApply{2ns};

  public: std::mutex renderMutex;
  public: std::condition_variable renderCv;
  public: std::mutex renderUtilMutex;
  public: std::condition_variable updateTimeCv;

  public: std::atomic<bool> waiterEntered{false};
  public: std::atomic<bool> waiterReturned{false};
  public: std::atomic<bool> updatedAfterShutdown{false};
  public: std::thread renderThread;

  public: void StartRenderThread()
  {
    this->renderThread = std::thread([this]
    {
      std::unique_lock<std::mutex> lock(this->renderMutex);
      this->renderCv.wait(lock, [this]
      {
        return !this->running;
      });
    });
  }

  // Mirrors the updateTimeCv wait in Sensors::PostUpdate().
  public: void PostUpdateWait()
  {
    std::unique_lock<std::mutex> lock(this->renderUtilMutex);
    this->waiterEntered = true;
    this->updateTimeCv.wait(lock, [this]
    {
#if defined(WITH_SHUTDOWN_FIX) || defined(WITH_NAIVE_SHUTDOWN_FIX)
      return !this->running || !this->updateAvailable ||
#else
      return !this->updateAvailable ||
#endif
             (this->updateTimeToApply == this->updateTimeApplied);
    });

#if defined(WITH_SHUTDOWN_FIX) || defined(WITH_NAIVE_SHUTDOWN_FIX)
    if (!this->running)
    {
      this->waiterReturned = true;
      return;
    }
#endif

    this->updatedAfterShutdown = !this->running;
    this->waiterReturned = true;
  }

  // Mirrors SensorsPrivate::Stop().
  public: void Stop()
  {
#ifdef WITH_SHUTDOWN_FIX
    // Quiesce an in-progress PostUpdate and make the predicate transition and
    // notification atomic with respect to its condition-variable wait.
    std::unique_lock<std::mutex> updateLock(this->renderUtilMutex);
#endif
    std::unique_lock<std::mutex> lock(this->renderMutex);
    this->running = false;
    lock.unlock();
    this->renderCv.notify_all();
#ifdef WITH_SHUTDOWN_FIX
    this->updateTimeCv.notify_all();
    updateLock.unlock();
#elif defined(WITH_NAIVE_SHUTDOWN_FIX)
    this->updateTimeCv.notify_all();
#endif
    if (this->renderThread.joinable())
      this->renderThread.join();
  }
};

int main()
{
  SensorsShutdownProtocol protocol;
  protocol.StartRenderThread();

  std::thread postUpdate([&protocol]
  {
    protocol.PostUpdateWait();
  });

  while (!protocol.waiterEntered)
    std::this_thread::yield();

  protocol.Stop();

  const auto deadline = std::chrono::steady_clock::now() + 100ms;
  while (!protocol.waiterReturned && std::chrono::steady_clock::now() < deadline)
    std::this_thread::yield();

  const bool unblockedByStop = protocol.waiterReturned;
  const bool touchedStateAfterStop = protocol.updatedAfterShutdown;

  // Cleanup only: make the old predicate true so a red run can terminate.
  if (!unblockedByStop)
  {
    std::unique_lock<std::mutex> lock(protocol.renderUtilMutex);
    protocol.updateAvailable = false;
    protocol.updateTimeCv.notify_all();
  }
  postUpdate.join();

  if (!unblockedByStop)
  {
    std::cerr << "FAIL: Stop joined the render thread but PostUpdate remained "
              << "blocked on updateTimeCv\n";
    return 1;
  }
  if (touchedStateAfterStop)
  {
    std::cerr << "FAIL: PostUpdate continued into update work after shutdown\n";
    return 2;
  }

  std::cout << "PASS: shutdown woke PostUpdate and it returned without update work\n";
  return 0;
}
