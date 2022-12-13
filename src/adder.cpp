#include "adder.hpp"


Adder::Adder(const int &a, const int &b) : a_(a), b_(b) {}

int Adder::add() const { return a_ + b_; }