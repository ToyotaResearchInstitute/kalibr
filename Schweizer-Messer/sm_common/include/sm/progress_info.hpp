#ifndef PROGRESS_INFO_HPP_
#define PROGRESS_INFO_HPP_

#include <cstddef>
#include <stdexcept>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

#include <sm/assert_macros.hpp>

namespace sm {

    void showProgress(double progress)
    {
        SM_ASSERT_GE_DBG(std::runtime_error, progress, 0.0, "negative progress! shame on you!");
        SM_ASSERT_LE_DBG(std::runtime_error, progress, 1.0, "progress out of bounds!");

        std::cout << std::fixed << std::setprecision(2) << std::showpoint;
        if(progress >= 1.0)
        {
            std::cout << "\r" << "Complete!             " << std::endl;
        } else
        {
            std::cout << "\r" << 100 * progress << "% complete ";
            std::cout.flush();
        }
    }

    template<typename T1, typename T2>
    void showProgress(T1 done, T2 all)
    {
        SM_ASSERT_GT(std::runtime_error, all, 0, "#DIV0");
        showProgress(static_cast<double>(done) / static_cast<double>(all));
    }

} /* namespace sm */

#endif /* PROGRESS_INFO_HPP_ */
