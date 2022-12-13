#include <catch2/catch_test_macros.hpp>
#include "../src/adder.hpp"

TEST_CASE("Adder of two int", "[Adder]")
{
  auto adder = Adder(12, 13);
  REQUIRE(adder.add() == 25);
}
