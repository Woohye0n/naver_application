//
// NOTE: THIS CODE IS FROM https://github.com/yongyanghz/LAPJV-algorithm-c
// with few adjustment.
// If copying this code is problem, I need to find other solution or implement LAP solver from the scratch.
//

#include "rr_object_tracker/lap.h"
#include <limits>

namespace
{
    template <typename T>
    inline bool max(const T &a, const T &b) { return a > b ? a : b; }
    template <typename T>
    inline bool min(const T &a, const T &b) { return a < b ? a : b; }
}

namespace rr_perception
{
    LAPJVAlgorithm::LAPJVAlgorithm() {}
    LAPJVAlgorithm::~LAPJVAlgorithm() {}
    bool LAPJVAlgorithm::Solve(const Eigen::MatrixXf &cost_mat)
    {
        // big value for initializing min
        static const float float_max = std::numeric_limits<float>::max();

        const int &rows = cost_mat.rows();
        const int &cols = cost_mat.cols();

        //early return for invalid problem
        if (rows == 0 || cols == 0)
        {
            return false;
        }

        // get dimension of the problem
        int dim = rows;
        if (dim < cols)
        {
            dim = cols;
        }
        // reserve output memory
        row_sol_.resize(dim, 0);
        col_sol_.resize(dim, 0);
        u_.resize(dim, 0.0);
        v_.resize(dim, 0.0);

        // make square cost matrix.
        Eigen::MatrixXf sq_cost_mat(dim, dim);
        if (rows == cols)
        {
            sq_cost_mat = cost_mat;
        }
        else
        {
            const float &min_cost = cost_mat.minCoeff();
            const float &max_cost = cost_mat.maxCoeff();
            const float &big_initial_cost = min_cost + (max_cost - min_cost) * 3.0;
            if (rows > cols)
            {
                sq_cost_mat << cost_mat, Eigen::MatrixXf::Ones(dim, dim - cols) * big_initial_cost;
            }
            else
            {
                sq_cost_mat << cost_mat, Eigen::MatrixXf::Ones(dim - rows, cols) * big_initial_cost;
            }
        }

        bool unassignedfound;
        int i, prvnumfree, f, i0, k, freerow;
        int j, j1, j2, endofpath, last, low, up;
        float min, h, umin, usubmin, v2;

        std::vector<int> free(dim, 0);    // list of unassigned rows.
        std::vector<int> collist(dim, 0); // list of columns to be scanned in various ways.
        std::vector<int> matches(dim, 0); // counts how many times a row could be assigned.
        std::vector<int> pred(dim, 0);    // row-predecessor of column in augmenting/alternating path.
        std::vector<float> d(dim, 0.0);   // 'cost-distance' in augmenting path calculation.

        int numfree = 0;

        // COLUMN REDUCTION
        for (j = dim; j--;) // reverse order gives better results.
        {
            // find minimum cost over rows.
            min = sq_cost_mat(0, j); // assigncost[0][j];
            int min_index = 0;
            for (i = 1; i < dim; i++)
            {
                const float &val = sq_cost_mat(i, j); //assigncost[i][j]
                if (val < min)
                {
                    min = val;
                    min_index = i;
                }
            }
            v_[j] = min;
            if (++matches[min_index] == 1)
            {
                // init assignment if minimum row assigned for first time.
                row_sol_[min_index] = j;
                col_sol_[j] = min_index;
            }
            else if (v_[j] < v_[row_sol_[min_index]])
            {
                j1 = row_sol_[min_index];
                row_sol_[min_index] = j;
                col_sol_[j] = min_index;
                col_sol_[j1] = -1;
            }
            else
            {
                col_sol_[j] = -1; // row already assigned, column not assigned.
            }
        }

        // REDUCTION TRANSFER
        for (i = 0; i < dim; i++)
        {
            if (matches[i] == 0)
            {
                // fill list of unassigned 'free' rows.
                free[numfree++] = i;
            }
            else if (matches[i] == 1) // transfer reduction from rows that are assigned once.
            {
                j1 = row_sol_[i];
                min = float_max;
                for (int j = 0; j < dim; j++)
                {
                    if (j != j1)
                    {
                        const float &r = sq_cost_mat(i, j) - v_[j];
                        if (r < min) //(assigncost[i][j] - v[j] < min)
                        {
                            min = r; // min = assigncost[i][j] - v[j];
                        }
                    }
                }
                v_[j1] = v_[j1] - min;
            }
        }

        //   AUGMENTING ROW REDUCTION
        int loopcnt = 0; // do-loop to be done twice.
        do
        {
            loopcnt++;
            //     scan all free rows.
            //     in some cases, a free row may be replaced with another one to be scanned next.
            k = 0;
            prvnumfree = numfree;
            numfree = 0; // start list of rows still free after augmenting row reduction.
            while (k < prvnumfree)
            {
                i = free[k];
                k++;
                // find minimum and second minimum reduced cost over columns.
                umin = sq_cost_mat(i, 0) - v_[0]; // assigncost[i][0] - v[0];
                j1 = 0;
                usubmin = float_max;
                for (j = 1; j < dim; j++)
                {
                    h = sq_cost_mat(i, j) - v_[j]; // assigncost[i][j] - v[j];
                    if (h < usubmin)
                    {
                        if (h >= umin)
                        {
                            usubmin = h;
                            j2 = j;
                        }
                        else
                        {
                            usubmin = umin;
                            umin = h;
                            j2 = j1;
                            j1 = j;
                        }
                    }
                }

                i0 = col_sol_[j1];
                if (umin < usubmin)
                {
                    //         change the reduction of the minimum column to increase the minimum
                    //         reduced cost in the row to the subminimum.
                    v_[j1] = v_[j1] - (usubmin - umin);
                }
                else             // minimum and subminimum equal.
                    if (i0 > -1) // minimum column j1 is assigned.
                {
                    //           swap columns j1 and j2, as j2 may be unassigned.
                    j1 = j2;
                    i0 = col_sol_[j2];
                }

                //       (re-)assign i to j1, possibly de-assigning an i0.
                row_sol_[i] = j1;
                col_sol_[j1] = i;

                if (i0 > -1)
                { // minimum column j1 assigned earlier.
                    if (umin < usubmin)
                    {
                        //           put in current k, and go back to that k.
                        //           continue augmenting path i - j1 with i0.
                        free[--k] = i0;
                    }
                    else
                    {
                        //           no further augmenting reduction possible.
                        //           store i0 in list of free rows for next phase.
                        free[numfree++] = i0;
                    }
                }
            }
        } while (loopcnt < 2); // repeat once.

        // AUGMENT SOLUTION for each free row.
        for (f = 0; f < numfree; f++)
        {
            freerow = free[f]; // start row of augmenting path.

            // Dijkstra shortest path algorithm.
            // runs until unassigned column added to shortest path tree.
            for (j = dim; j--;)
            {
                d[j] = sq_cost_mat(freerow, j) - v_[j]; //assigncost[freerow][j] - v[j];
                pred[j] = freerow;
                collist[j] = j; // init column list.
            }

            low = 0; // columns in 0..low-1 are ready, now none.
            up = 0;  // columns in low..up-1 are to be scanned for current minimum, now none.
                     // columns in up..dim-1 are to be considered later to find new minimum,
                     // at this stage the list simply contains all columns
            unassignedfound = false;
            do
            {
                if (up == low) // no more columns to be scanned for current minimum.
                {
                    last = low - 1;

                    // scan columns for up..dim-1 to find all indices for which new minimum occurs.
                    // store these indices between low..up-1 (increasing up).
                    min = d[collist[up++]];
                    for (k = up; k < dim; k++)
                    {
                        j = collist[k];
                        h = d[j];
                        if (h <= min)
                        {
                            if (h < min) // new minimum.
                            {
                                up = low; // restart list at index low.
                                min = h;
                            }
                            // new index with same minimum, put on undex up, and extend list.
                            collist[k] = collist[up];
                            collist[up++] = j;
                        }
                    }
                    // check if any of the minimum columns happens to be unassigned.
                    // if so, we have an augmenting path right away.
                    for (k = low; k < up; k++)
                        if (col_sol_[collist[k]] < 0)
                        {
                            endofpath = collist[k];
                            unassignedfound = true;
                            break;
                        }
                }

                if (!unassignedfound)
                {
                    // update 'distances' between freerow and all unscanned columns, via next scanned column.
                    j1 = collist[low];
                    low++;
                    i = col_sol_[j1];
                    h = sq_cost_mat(i, j1) - v_[j1] - min; //assigncost[i][j1] - v[j1] - min;

                    for (k = up; k < dim; k++)
                    {
                        j = collist[k];
                        v2 = sq_cost_mat(i, j) - v_[j] - h; // assigncost[i][j] - v[j] - h;
                        if (v2 < d[j])
                        {
                            pred[j] = i;
                            if (v2 == min) // new column found at same minimum value
                                if (col_sol_[j] < 0)
                                {
                                    // if unassigned, shortest augmenting path is complete.
                                    endofpath = j;
                                    unassignedfound = true;
                                    break;
                                }
                                // else add to list to be scanned right away.
                                else
                                {
                                    collist[k] = collist[up];
                                    collist[up++] = j;
                                }
                            d[j] = v2;
                        }
                    }
                }
            } while (!unassignedfound);

            // update column prices.
            for (k = last + 1; k--;)
            {
                j1 = collist[k];
                v_[j1] = v_[j1] + d[j1] - min;
            }

            // reset row and column assignments along the alternating path.
            do
            {
                i = pred[endofpath];
                col_sol_[endofpath] = i;
                j1 = endofpath;
                endofpath = row_sol_[i];
                row_sol_[i] = j1;
            } while (i != freerow);
        }

        // calculate optimal cost.
        float lapcost = 0;
        //  for (i = 0; i < dim; i++)
        for (i = dim; i--;)
        {
            j = row_sol_[i];
            u_[i] = sq_cost_mat(i, j) - v_[j];     //assigncost[i][j] - v[j];
            lapcost = lapcost + sq_cost_mat(i, j); //assigncost[i][j];
        }
        optimal_cost_ = lapcost;

        for (int i = 0; i < row_sol_.size(); ++i)
        {
            if ((i < rows) && (row_sol_[i] < cols))
            {
                r2c_.push_back({i, row_sol_[i]});
            }
        }
        return true;
    }

    std::vector<IndicesPair> &LAPJVAlgorithm::Solution()
    {
        return r2c_;
    }

} // namespace rr_object_tracking