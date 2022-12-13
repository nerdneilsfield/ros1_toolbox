#ifndef __ADDER_HPP__
#define __ADDER_HPP__

class Adder {
public:
    Adder(const int& a, const int& b);
    int add() const;
private:
    int a_, b_;
};


#endif // __ADDER_HPP__