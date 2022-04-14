//
// Created by micha on 4/12/2022.
//

#pragma once

#include <vector>
#include <cmath>


class StatisticalAnalysis {

public:

    template <typename T>
    static double get_standard_deviation(const std::vector<T> &array){

        double mean = get_mean(array);

        T sum_of_squares = 0;

        for (const T &item : array) {
            sum_of_squares += (item - mean) * (item - mean);
        }

        return sqrt(sum_of_squares / (array.size() - 1.0));
    }


    template <typename T>
    static double get_mean(const std::vector<T> &array) {

        double sum_of_elements = 0.0;

        for (const T &item : array) {
            sum_of_elements += item;
        }

        return sum_of_elements / array.size();
    }


};


