#pragma once
#include <bit>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <openssl/evp.h>
#include "rm_r4_prediction_consumption/consumption.hpp"

namespace rm_r4_prediction_consumption
{
// Length-prefixed strings, little-endian integers and IEEE754 binary64 values.
// Exact input content is retained; this is identity, never a safety certificate.
class Digest
{
public:
  void integer(uint64_t value)
  {
    for (int i = 0; i < 8; ++i) {data_.push_back(static_cast<unsigned char>(value >> (8 * i)));}
  }
  void number(double value) {integer(std::bit_cast<uint64_t>(value));}
  void text(const std::string & value)
  {
    integer(value.size()); data_.insert(data_.end(), value.begin(), value.end());
  }
  void point(Vec2 value) {number(value.x); number(value.y);}
  std::string finish() const
  {
    unsigned char output[EVP_MAX_MD_SIZE]; unsigned int size = 0;
    if (EVP_Digest(data_.data(), data_.size(), output, &size, EVP_sha256(), nullptr) != 1) {
      throw ContractError("content digest failed");
    }
    std::ostringstream out;
    for (unsigned int i = 0; i < size; ++i) {
      out << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(output[i]);
    }
    return out.str();
  }
private:
  std::vector<unsigned char> data_;
};
}  // namespace rm_r4_prediction_consumption
