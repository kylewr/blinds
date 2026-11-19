#pragma once

#include <cstdint>

class ConsecutiveFailureLatch {
 public:
  explicit ConsecutiveFailureLatch(std::uint8_t limit) : limit_(limit) {}

  void failure() {
    if (count_ < 255) ++count_;
    if (count_ >= limit_) latched_ = true;
  }

  void success() { count_ = 0; }
  void clear() {
    count_ = 0;
    latched_ = false;
  }

  std::uint8_t count() const { return count_; }
  bool latched() const { return latched_; }

 private:
  const std::uint8_t limit_;
  std::uint8_t count_ = 0;
  bool latched_ = false;
};
