
#pragma once

namespace SmallRobots {

    template<typename T>
    class Hysteresis {
    public:
        Hysteresis(T threshold, T hysteresis, T initial_value) : t(threshold), h(hysteresis), less(initial_value<threshold) {};
        friend bool operator<(const T value, Hysteresis<T>& o) {
            if (o.less) {
                if (value > o.t + o.h)
                    o.less = false;
            }
            else {
                if (value < o.t - o.h)
                    o.less = true;
            }
            return o.less;
        };
        friend bool operator>(const T value, Hysteresis<T>& o) {
            if (o.less) {
                if (value > o.t + o.h)
                    o.less = false;
            }
            else {
                if (value < o.t - o.h)
                    o.less = true;
            }
            return !o.less;
        };
    protected:
        T t;
        T h;
        bool less;
    };

};