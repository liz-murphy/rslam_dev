// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef _QUAD_TREE_H_
#define _QUAD_TREE_H_

#include <vector>

struct CellBounds {
  CellBounds():
      t(0),
      l(0),
      b(0),
      r(0) {}

  CellBounds(int top, int left, int bottom, int right):
      t(top),
      l(left),
      b(bottom),
      r(right) {}

  CellBounds(const CellBounds& RHS) {
    t = RHS.t;
    l = RHS.l;
    b = RHS.b;
    r = RHS.r;
  }

  int t;
  int l;
  int b;
  int r;
};

class QuadTree {
 public:
  QuadTree():
      num_levels_(0),
      num_cells_(0),
      image_width_(0),
      image_height_(0),
      num_desired_features_(0)
  {}

  ~QuadTree(){}

  ///
  /// \brief Init
  /// \param[in] image_width
  /// \param[in] image_height
  /// \param[in] num_levels
  /// \param[in] num_desired_features
  ///
  void Init(int image_width,
            int image_height,
            int num_levels,
            unsigned int num_desired_features);

  ///
  /// \brief AddPoint point to the tree (increment count in all cells)
  /// \param[in] row
  /// \param[in] col
  ///
  void AddPoint(int row, int col );

  ///
  /// \brief Levels
  /// \return num levels
  ///
  int Levels() { return num_levels_; }

  ///
  /// \brief Print quad tree fill pattern
  ///
  void PrintTree();

  // Given a cell, return it's image bounds
  CellBounds& GetCellBounds(int cell_idx );

  // Return a list of the least populated leaves
  void GetMostlyEmptyCellList(std::vector<int>& cells, int *roi = nullptr );

  void DeactivateCells(std::vector<int>& cells);

  int GetCellIndex(int row, int col, int level);

  int GetCellIndexRecursive(int cell_index,
                            int cell_row,
                            int cell_col,
                            int level,
                            int goal_cell_row,
                            int goal_cell_col,
                            int goal_level );


 private:
  void ComputeCellBounds();

  void ComputeCellChildrenBoundsRecursively(int cell_idx );

  bool DeactivateParentCellsRecursively(int cell_idx );

  void AddPointRecursively(int cell_idx, int level, int eor, int col );

  void FindMostlyEmptyCellsRecursively(int cell_idx,
                                       std::vector<int>& cells,
                                       int *roi);

  // First element is coarsest level, next 4 are level 1, next 16 are level 2...
  std::vector<int>  cell_feature_count_;
  std::vector<int>  max_feats_in_cell_per_level_;
  std::vector<bool> active_cells_;
  // how many levels in the quad tree
  int                           num_levels_;
  int               num_cells_;
  int                           image_width_;
  int                           image_height_;
  // we distribute these over the quad tree
  int               num_desired_features_;

  std::vector<CellBounds>  cells_bounds_;
};

#endif
