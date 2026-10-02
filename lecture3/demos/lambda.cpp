#include <iostream>

struct Tracker
{
  int count;
};

int main()
{
  int frame_id = 10;
  Tracker tracker{1};

  // Create the lambda without invoking it yet.
  auto task = [frame_id, &tracker](int delta) {
    std::cout << "frame_id = " << frame_id << '\n';
    std::cout << "delta = " << delta << '\n';

    tracker.count += delta;
    std::cout << "tracker.count = " << tracker.count << '\n';
  };

  frame_id = 99;
  tracker.count = 7;

  task(3);
}
