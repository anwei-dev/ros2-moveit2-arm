#ifndef COLOR_POINT_CLOUD_DETECTOR__COLOR_CLASSIFIER_HPP_
#define COLOR_POINT_CLOUD_DETECTOR__COLOR_CLASSIFIER_HPP_

#include <cstdint>

enum class ColorLabel
{
  kUnknown,
  kRed,
  kBlue
};

class ColorClassifier
{
public:
  ColorLabel classify(float rgb_float) const;

  static ColorLabel dominantColor(
    std::size_t red_count,
    std::size_t blue_count,
    std::size_t unknown_count);

  static const char *toString(ColorLabel color);
};

#endif