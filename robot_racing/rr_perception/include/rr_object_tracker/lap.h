//
// NOTE: THIS CODE IS FROM https://github.com/yongyanghz/LAPJV-algorithm-c
// with few adjustment.
// If copying this code is problem, I need to find other solution or implement LAP solver from the scratch.
//

#ifndef RR_PERCEPTION_RR_OBJECT_TRACKER_LAP_H_
#define RR_PERCEPTION_RR_OBJECT_TRACKER_LAP_H_

#include <Eigen/Dense>
#include <vector>

namespace rr_perception
{
    struct IndicesPair
    {
        int row;
        int col;
    };
    class LAPJVAlgorithm
    {
    public:
        LAPJVAlgorithm();
        ~LAPJVAlgorithm();
        bool Solve(const Eigen::MatrixXf &cost_mat);
        std::vector<IndicesPair> &Solution();

    private:
        std::vector<int> row_sol_;
        std::vector<int> col_sol_;
        std::vector<float> u_;
        std::vector<float> v_;
        float optimal_cost_;

        std::vector<IndicesPair> r2c_;
    };
}

#endif //RR_PERCEPTION_RR_OBJECT_TRACKER_LAP_H_