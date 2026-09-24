#include <gtest/gtest.h>
#include "rm_dynamic_prediction_critic/geometry.hpp"
using namespace rm_dynamic_prediction_critic;
TEST(Prediction, HiddenFaceAndAge) {
  auto b=predicted_box({0,0},{1,0},{.2,.1},{.45,.55},.1,.2,.5);
  EXPECT_NEAR(b.min_x,.3-.1-.45-.5*.5*.3*.3,1e-12);
  EXPECT_NEAR(b.max_y,.05+.55+.5*.5*.3*.3,1e-12);
}
TEST(Prediction, InvalidInputRejected) {
  EXPECT_THROW(predicted_box({0,0},{0,0},{0,0},{.45,.55},-.1,.2,0),std::invalid_argument);
  EXPECT_THROW(predicted_box({NAN,0},{0,0},{0,0},{.45,.55},0,.2,0),std::invalid_argument);
}
TEST(Geometry, RotatedPolygonContactAndClearance) {
  const std::vector<Point> square{{-.5,-.5},{.5,-.5},{.5,.5},{-.5,.5}};
  EXPECT_EQ(polygon_box_distance(transform(square,0,0,0),{.5,-.1,.8,.1}),0);
  EXPECT_NEAR(polygon_box_distance(transform(square,0,0,0),{.7,-.1,.8,.1}),.2,1e-12);
  EXPECT_EQ(polygon_box_distance(transform(square,0,0,M_PI/4),{.6,-.1,.8,.1}),0);
}
