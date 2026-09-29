#include "color_classifier.hpp"

#include <algorithm>
#include <cstring>

ColorLabel ColorClassifier::classify(float rgb_float) const
{
  std::uint32_t rgb_packed = 0;

  std::memcpy(
    &rgb_packed,
    &rgb_float,
    sizeof(float));

  const std::uint8_t r =
    static_cast<std::uint8_t>(
      (rgb_packed >> 16) & 0xFF);

  const std::uint8_t g =
    static_cast<std::uint8_t>(
      (rgb_packed >> 8) & 0xFF);

  const std::uint8_t b =
    static_cast<std::uint8_t>(
      rgb_packed & 0xFF);

  const std::uint8_t max_channel =
    std::max({r, g, b});

  if (max_channel < 50)
  {
    return ColorLabel::kUnknown;
  }

  if (r >= 90 &&
      r > static_cast<std::uint8_t>(g * 1.25) &&
      r > static_cast<std::uint8_t>(b * 1.25))
  {
    return ColorLabel::kRed;
  }

  if (b >= 90 &&
      b > static_cast<std::uint8_t>(r * 1.25) &&
      b > static_cast<std::uint8_t>(g * 1.10))
  {
    return ColorLabel::kBlue;
  }

  return ColorLabel::kUnknown;
}

ColorLabel ColorClassifier::dominantColor(
  std::size_t red_count,
  std::size_t blue_count,
  std::size_t unknown_count)
{
  (void)unknown_count;

  if (red_count == 0 && blue_count == 0)
  {
    return ColorLabel::kUnknown;
  }

  return red_count >= blue_count
    ? ColorLabel::kRed
    : ColorLabel::kBlue;
}

const char *ColorClassifier::toString(ColorLabel color)
{
  switch (color)
  {
    case ColorLabel::kRed:
      return "red";

    case ColorLabel::kBlue:
      return "blue";

    case ColorLabel::kUnknown:
    default:
      return "unknown";
  }
}