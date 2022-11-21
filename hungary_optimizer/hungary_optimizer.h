#ifndef HUNGARY_OPTIMIZER_H__
#define HUNGARY_OPTIMIZER_H__

#include <eigen3/Eigen/Core>
#include <iostream>
#include <vector>

class HungaryOptimizer {
 public:
  enum ZeroType { Star = 1, Primed };

  HungaryOptimizer() { clearFlags(); }

  bool minimize(const Eigen::MatrixXd &cost_matrix, std::vector<int> *results, bool print_log = false) {
    if (results == nullptr) {
      std::cout << "invalid results ptr! " << std::endl;
      return false;
    }
    clearFlags();
    if (!preprocess(cost_matrix)) {
      std::cout << "cannot set cost matrix!" << std::endl;
      return false;
    }
    doMunkres();

    results->resize(num_row_, -1);
    for (size_t r = 0; r < num_row_; ++r) {
      for (size_t c = 0; c < num_col_; ++c) {
        if (markers_(r, c) == ZeroType::Star) {
          results->at(r) = c;
        }
      }
    }
    if (print_log) {
      std::cout << "best matches are: ";
      for (size_t r = 0; r < num_row_; ++r) {
        std::cout << results->at(r) << ", ";
      }
      std::cout << std::endl;
    }
    return true;
  }

 private:
  void clearFlags() {
    done_ = false;
    step_ = 1;
    print_log_ = false;
  }

  bool preprocess(const Eigen::MatrixXd &cost_matrix) {
    num_row_ = cost_matrix.rows();
    num_col_ = cost_matrix.cols();
    n_ = std::max(num_row_, num_col_);
    cost_matrix_.resize(n_, n_);
    cost_matrix_.topLeftCorner(num_row_, num_col_) = cost_matrix;
    markers_.resize(n_, n_);
    markers_.setZero();
    row_cover_.resize(n_, 0);
    col_cover_.resize(n_, 0);
    return true;
  }

  // Please refer to https://brc2.com/the-algorithm-workshop/
  void doMunkres() {
    while (!done_) {
      if (print_log_) {
        std::cout << "---------------------" << std::endl;
        std::cout << "execute step: " << step_ << std::endl;
      }
      switch (step_) {
      case 1:
        stepOne();
        break;
      case 2:
        stepTwo();
        break;
      case 3:
        stepThree();
        break;
      case 4:
        stepFour();
        break;
      case 5:
        stepFive();
        break;
      case 6:
        stepSix();
        break;

      default:
        std::cout << "invalid step " << step_ << ", exit" << std::endl;
        done_ = true;
        break;
      }
      if (print_log_) {
        std::cout << "cost matrix: " << std::endl;
        std::cout << cost_matrix_ << std::endl;

        std::cout << "markers: " << std::endl;
        std::cout << markers_ << std::endl;

        std::cout << "row_cover: " << std::endl;
        for (size_t ci = 0; ci < n_; ++ci) {
          std::cout << row_cover_[ci] << ", ";
        }
        std::cout << std::endl;

        std::cout << "col_cover: " << std::endl;
        for (size_t ci = 0; ci < n_; ++ci) {
          std::cout << col_cover_[ci] << ", ";
        }
        std::cout << std::endl;
      }
    }
  }

  // For each row of the matrix, find the smallest element and subtract it from every element in its row.  Go to Step 2.
  void stepOne() {
    for (int r = 0; r < n_; ++r) {
      double min_in_row = cost_matrix_.row(r).minCoeff();
      cost_matrix_.row(r) -= min_in_row * Eigen::MatrixXd::Ones(n_, 1);
    }
    step_ = 2;
  }

  // Find a zero (Z) in the resulting matrix.  If there is no starred zero in its row or column, star Z. Repeat for each
  // element in the matrix. Go to Step 3.
  void stepTwo() {
    for (int r = 0; r < n_; ++r) {
      for (int c = 0; c < n_; ++c) {
        if (cost_matrix_(r, c) == 0 && row_cover_[r] == 0 && col_cover_[c] == 0) {
          markers_(r, c) = 1;
          row_cover_[r] = 1;
          col_cover_[c] = 1;
        }
      }
    }
    std::fill(row_cover_.begin(), row_cover_.end(), 0);
    std::fill(col_cover_.begin(), col_cover_.end(), 0);
    step_ = 3;
  }

  // Cover each column containing a starred zero.  If K columns are covered, the starred zeros describe a complete set
  // of unique assignments.  In this case, Go to DONE, otherwise, Go to Step 4.
  void stepThree() {
    double cover_cnt = 0;
    for (size_t c = 0; c < n_; ++c) {
      for (size_t r = 0; r < n_; ++r) {
        if (markers_(r, c) == 1) {
          col_cover_[c] = 1;
          ++cover_cnt;
          break;
        }
      }
    }
    if (cover_cnt == n_) {
      done_ = true;
    } else {
      step_ = 4;
    }
  }

  // Find a noncovered zero and prime it.  If there is no starred zero in the row containing this primed zero, Go to
  // Step 5.  Otherwise, cover this row and uncover the column containing the starred zero. Continue in this manner
  // until there are no uncovered zeros left. Save the smallest uncovered value and Go to Step 6.
  void stepFour() {
    bool finished = false;
    while (!finished) {
      int r_primed0, c_primed0;
      if (!findANonCoveredZero(&r_primed0, &c_primed0)) {
        finished = true;
        step_ = 6;
      } else {
        markers_(r_primed0, c_primed0) = 2;  // mark as prime
        int c_star0;
        if (findSpecificZero(r_primed0, &c_star0, ZeroType::Star)) {
          row_cover_[r_primed0] = 1;
          col_cover_[c_star0] = 0;
        } else {
          finished = true;
          step_ = 5;
          path_row_0_ = r_primed0;
          path_col_0_ = c_primed0;
        }
      }
    }
  }

  // Construct a series of alternating primed and starred zeros as follows.  Let Z0 represent the uncovered primed zero
  // found in Step 4.  Let Z1 denote the starred zero in the column of Z0 (if any). Let Z2 denote the primed zero in the
  // row of Z1 (there will always be one).  Continue until the series terminates at a primed zero that has no starred
  // zero in its column.  Unstar each starred zero of the series, star each primed zero of the series, erase all primes
  // and uncover every line in the matrix.  Return to Step 3.
  void stepFive() {
    bool finished = false;
    std::vector<std::pair<int, int>> path(1);
    int path_count = 1;
    path[path_count - 1].first = path_row_0_;
    path[path_count - 1].second = path_col_0_;
    while (!finished) {
      int star_zero_row = 0;
      if (findSpecificZero(&star_zero_row, path[path_count - 1].second, ZeroType::Star)) {
        // std::cout << "find star zero at : " << star_zero_row << ", " << path[path_count - 1].second << std::endl;
        ++path_count;
        path.emplace_back();
        path.back().first = star_zero_row;
        path.back().second = path[path_count - 2].second;
      } else {
        finished = true;
      }
      if (!finished) {
        int primed_zero_col = 0;
        findSpecificZero(path[path_count - 1].first, &primed_zero_col, ZeroType::Primed);
        // std::cout << "find primed zero at : " << path[path_count - 1, ]  << ", " << primed_zero_col << std::endl;
        ++path_count;
        path.emplace_back();
        path.back().first = path[path_count - 2].first;
        path.back().second = primed_zero_col;
      }
    }

    augmentPath(path);
    clearCovers();
    erasePrimes();
    step_ = 3;
  }

  // Add the value found in Step 4 to every element of each covered row, and subtract it from every element of each
  // uncovered column.  Return to Step 4 without altering any stars, primes, or covered lines.
  void stepSix() {
    double min_val = std::numeric_limits<double>::max();
    for (size_t r = 0; r < n_; ++r) {
      for (size_t c = 0; c < n_; ++c) {
        if (row_cover_[r] == 0 && col_cover_[c] == 0) {
          min_val = std::min(min_val, cost_matrix_(r, c));
        }
      }
    }

    for (size_t r = 0; r < n_; ++r) {
      for (size_t c = 0; c < n_; ++c) {
        if (row_cover_[r] == 1) {
          cost_matrix_(r, c) += min_val;
        }
        if (col_cover_[c] == 0) {
          cost_matrix_(r, c) -= min_val;
        }
      }
    }
    step_ = 4;
  }

  void augmentPath(const std::vector<std::pair<int, int>> &path) {
    for (size_t p = 0; p < path.size(); ++p) {
      if (markers_(path[p].first, path[p].second) == 1) {
        markers_(path[p].first, path[p].second) = 0;
      } else {
        markers_(path[p].first, path[p].second) = 1;
      }
    }
  }

  void clearCovers() {
    for (size_t c = 0; c < n_; ++c) {
      row_cover_[c] = 0;
      col_cover_[c] = 0;
    }
  }

  void erasePrimes() {
    for (int r = 0; r < n_; ++r) {
      for (int c = 0; c < n_; ++c) {
        if (markers_(r, c) == 2) {
          markers_(r, c) == 0;
        }
      }
    }
  }

  bool findANonCoveredZero(int *row, int *col) {
    for (size_t r = 0; r < n_; ++r) {
      for (size_t c = 0; c < n_; ++c) {
        if (std::fabs(cost_matrix_(r, c)) < 1e-5 && row_cover_[r] == 0 && col_cover_[c] == 0) {
          *row = r;
          *col = c;
          return true;
        }
      }
    }
    return false;
  };

  template <typename TRow, typename TCol, typename std::enable_if<std::is_pointer<TRow>::value>::type * = nullptr,
            typename std::enable_if<std::is_pointer<TCol>::value>::type * = nullptr>
  bool findSpecificZero(TRow row, TCol col, ZeroType type) {
    for (size_t r = 0; r < n_; ++r) {
      for (size_t c = 0; c < n_; ++c) {
        if (markers_(r, c) == type) {
          *row = r;
          *col = c;
          return true;
        }
      }
    }
    return false;
  }

  template <typename TRow, typename TCol, typename std::enable_if<!std::is_pointer<TRow>::value>::type * = nullptr,
            typename std::enable_if<std::is_pointer<TCol>::value>::type * = nullptr>
  bool findSpecificZero(TRow row, TCol col, ZeroType type) {
    for (int k = 0; k < n_; ++k) {
      if (markers_(row, k) == type) {
        *col = k;
        return true;
      }
    }
    return false;
  }

  template <typename TRow, typename TCol, typename std::enable_if<std::is_pointer<TRow>::value>::type * = nullptr,
            typename std::enable_if<!std::is_pointer<TCol>::value>::type * = nullptr>
  bool findSpecificZero(TRow row, TCol col, ZeroType type) {
    for (size_t k = 0; k < n_; ++k) {
      if (markers_(k, col) == type) {
        *row = k;
        return true;
      }
    }
    return false;
  }

  template <typename TRow, typename TCol, typename std::enable_if<!std::is_pointer<TRow>::value>::type * = nullptr,
            typename std::enable_if<!std::is_pointer<TCol>::value>::type * = nullptr>
  bool findSpecificZero(TRow row, TCol col, ZeroType type) {
    std::cout << "input row and col are all not pointer !" << std::endl;
    return false;
  }

  bool done_;
  int step_;
  Eigen::MatrixXd cost_matrix_;
  Eigen::MatrixXi markers_;
  std::vector<int> row_cover_, col_cover_;
  int num_row_;
  int num_col_;
  int path_row_0_;
  int path_col_0_;
  int n_;
  bool print_log_;
};

#endif  // HUNGARY_OPTIMIZER_H__
