#include "statistics.hpp"

#include <chrono>
#include <thread>

namespace
{
    void deliberatelySlowIncrement(int &value)
    {
        // This delay makes the race observable on small homework inputs.
        const int old = value;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        value = old + 1;
    }
}

void Statistics::onProduced() { deliberatelySlowIncrement(produced_); }
void Statistics::onProcessed() { deliberatelySlowIncrement(processed_); }
void Statistics::onSaved() { deliberatelySlowIncrement(saved_); }
void Statistics::onCorrupted() { deliberatelySlowIncrement(corrupted_); }

StatisticsSnapshot Statistics::snapshot() const
{
    return {produced_, processed_, saved_, corrupted_};
}
