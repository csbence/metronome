#ifndef METRONOME_STATISTIC_HPP
#define METRONOME_STATISTIC_HPP

#include <cmath>
namespace metronome {

class Statistic {
public:
    static double normalPDF(double mean, double standardDeviation, double x) {
        return standardNormalPDF((x - mean) / standardDeviation);
    }

    static double standardNormalPDF(double z) { return standardNormalCoefficient * std::exp(-z * z / 2); }

    //    double normalCDFBugby(double mean, double variance, double variable) const {
    //        double x = (variable - mean) / sqrt(variance);
    //        return 0.5 * pow(1.0 - 1.0 / 30.0 * (7 * std::exp(-x * x / 2) + 16 * std::exp(-x * x * (2 - sqrt(2.0))) +
    //        (7
    //                             + 0.25 * PI * x * x) * std::exp(-x * x)),
    //                         0.5) + 0.5;
    //
    //    }
    //
    //    double normalCDFLin1990(double mean, double variance, double variable) const {
    //        double x = (variable - mean) / sqrt(variance);
    //        double cumulative = 1.0 / (1.0 + std::exp((4.2 * PI * x) / (9.0 - x)));
    //        return cumulative;
    //    }
    //
    //    double normalCDFLin1989(double mean, double variance, double variable) const {
    //        double x = (variable - mean) / sqrt(variance);
    //        double cumulative = 0.5 * std::exp(-0.717 * x - 0.416 * x * x);
    //        return cumulative;
    //    }

    /**
     * Standard normal table for numerical integration.
     */
    class StandardNormalTable {
    public:
        StandardNormalTable(int resolution) : table{new double[resolution]} {
            const double step{6.0 / resolution}; // 6 = 2 * std dev

            int i{0};
            for (double x = -3; x < 3; x += step) {
                table[i] = (standardNormalPDF(x) + standardNormalPDF(x + step)) / 2 * step;
                ++i;
            }
        }

        ~StandardNormalTable() { delete[] table; }

        double operator[](int index) const { return table[index]; }

    private:
        double* table;
    };

    static double standardNormalTable100(int index) {
        static const StandardNormalTable table{100};
        return table[index];
    }

    static double standardNormalTable500(int index) {
        static const StandardNormalTable table{500};
        return table[index];
    }

    static void initialize() {
        LOG(INFO) << "Init";
        standardNormalTable100(0);
        standardNormalTable500(0);
    }

private:
    constexpr static const double PI{3.141592653589793};
    constexpr static double standardNormalCoefficient{0.39894228040143}; // 1/ sqrt(2*PI)
};

}

#endif // METRONOME_STATISTIC_HPP
