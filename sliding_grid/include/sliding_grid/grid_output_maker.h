#ifndef PMAP_MSG_CHECKER_GRID_OUTPUT_MAKER_H
#define PMAP_MSG_CHECKER_GRID_OUTPUT_MAKER_H

#include <array>
#include <sstream>
#include <string>
#include <vector>

template <size_t kCols>
class FixedGridOutputMaker {
 public:
  FixedGridOutputMaker() { std::fill(arr_max_length_per_col_.begin(), arr_max_length_per_col_.end(), 0); }

  void setTitle(const std::vector<std::string>& v_str) {
    if (v_str.size() != kCols) {
      throw std::logic_error("v_str size != col size.");
    }
    for (size_t ci = 0; ci < kCols; ++ci) {
      arr_title_strs_[ci]         = v_str[ci];
      arr_max_length_per_col_[ci] = std::max(v_str[ci].size(), arr_max_length_per_col_[ci]);
    }
  }

  void addRow(const std::vector<std::string>& v_str) {
    if (v_str.size() != kCols) {
      throw std::logic_error("v_str size != col size.");
    }
    for (size_t ci = 0; ci < kCols; ++ci) {
      arr_all_strs_[ci].emplace_back(v_str[ci]);
      arr_max_length_per_col_[ci] = std::max(v_str[ci].size(), arr_max_length_per_col_[ci]);
    }
  }

  std::string toString() {
    std::stringstream ss;

    for (size_t ci = 0; ci < kCols; ++ci) {
      ss << " | " << arr_title_strs_[ci];
      size_t curr_length = arr_title_strs_[ci].size();
      for (size_t si = 0; si < (arr_max_length_per_col_[ci] - curr_length); ++si) {
        ss << " ";
      }
    }
    ss << " | " << std::endl;

    size_t total_length = 0;
    for (size_t ci = 0; ci < kCols; ++ci) {
      total_length += arr_max_length_per_col_[ci];
    }
    total_length += (kCols + 1) * 3;
    for (size_t ti = 0; ti < total_length; ++ti) {
      ss << "-";
    }
    ss << std::endl;

    size_t rows = arr_all_strs_[0].size();
    for (size_t ri = 0; ri < rows; ++ri) {
      for (size_t ci = 0; ci < kCols; ++ci) {
        ss << " | " << arr_all_strs_[ci][ri];
        size_t curr_length = arr_all_strs_[ci][ri].size();
        for (size_t si = 0; si < (arr_max_length_per_col_[ci] - curr_length); ++si) {
          ss << " ";
        }
      }
      ss << " | " << std::endl;
    }
    return ss.str();
  }

 private:
  std::array<std::vector<std::string>, kCols> arr_all_strs_;
  std::array<std::string, kCols>              arr_title_strs_;
  std::array<size_t, kCols>                   arr_max_length_per_col_;
};

class DynamicGridOutputMaker {
 public:
  DynamicGridOutputMaker() {}

  void setTitle(const std::vector<std::string>& v_str) {
    cols_ = v_str.size();
    v_title_strs_.resize(cols_);
    v_max_length_per_col_.resize(cols_, 0);
    v_all_strs_.resize(cols_);
    for (size_t ci = 0; ci < cols_; ++ci) {
      v_title_strs_[ci]         = v_str[ci];
      v_max_length_per_col_[ci] = std::max(v_str[ci].size(), v_max_length_per_col_[ci]);
    }
  }

  void addRow(const std::vector<std::string>& v_str) {
    if (v_str.size() != cols_) {
      throw std::logic_error("v_str size != col size.");
    }
    for (size_t ci = 0; ci < cols_; ++ci) {
      v_all_strs_[ci].emplace_back(v_str[ci]);
      v_max_length_per_col_[ci] = std::max(v_str[ci].size(), v_max_length_per_col_[ci]);
    }
  }

  std::string toString() {
    std::stringstream ss;

    for (size_t ci = 0; ci < cols_; ++ci) {
      ss << " | " << v_title_strs_[ci];
      size_t curr_length = v_title_strs_[ci].size();
      for (size_t si = 0; si < (v_max_length_per_col_[ci] - curr_length); ++si) {
        ss << " ";
      }
    }
    ss << " | " << std::endl;

    size_t total_length = 0;
    for (size_t ci = 0; ci < cols_; ++ci) {
      total_length += v_max_length_per_col_[ci];
    }
    total_length += (cols_ + 1) * 3;
    for (size_t ti = 0; ti < total_length; ++ti) {
      ss << "-";
    }
    ss << std::endl;

    size_t rows = v_all_strs_[0].size();
    for (size_t ri = 0; ri < rows; ++ri) {
      for (size_t ci = 0; ci < cols_; ++ci) {
        ss << " | " << v_all_strs_[ci][ri];
        size_t curr_length = v_all_strs_[ci][ri].size();
        for (size_t si = 0; si < (v_max_length_per_col_[ci] - curr_length); ++si) {
          ss << " ";
        }
      }
      ss << " | " << std::endl;
    }
    return ss.str();
  }

 private:
  std::vector<std::vector<std::string>> v_all_strs_;
  std::vector<std::string>              v_title_strs_;
  std::vector<size_t>                   v_max_length_per_col_;
  int                                   cols_ = 0;
};

#endif  // PMAP_MSG_CHECKER_GRID_OUTPUT_MAKER_H
