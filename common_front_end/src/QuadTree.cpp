// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <common_front_end/QuadTree.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <limits.h>

void QuadTree::Init(int image_width,
                    int image_height,
                    int num_levels,
                    unsigned int num_desired_features) {
  num_desired_features_ = (int)num_desired_features;
  image_width_         = image_width;
  image_height_        = image_height;
  num_levels_          = num_levels;
  // number of cells in FeatureCount vector is
  // 1 + 4 + 4*4 + 4*4*4 + 4*4*4*4 ...
  // => sumation rule: sum_i=0^n r^i = (1-r^n)/(1-r)

  num_cells_ = (int)( ( 1 << 2*num_levels ) - 1 )/3;

  cell_feature_count_.clear();
  cell_feature_count_.resize( num_cells_, 0 );
  cells_bounds_.clear();
  cells_bounds_.resize( num_cells_ );
  active_cells_.clear();
  active_cells_.resize( num_cells_, true);

  ComputeCellBounds();

  max_feats_in_cell_per_level_.clear();

  for (int ii=0; ii < num_levels; ++ii) {
    int max_features =  (int)ceil((float)num_desired_features_ / (1 << 2*ii));

    if (max_features == 0) {
      max_features = 1;
    }

    max_feats_in_cell_per_level_.push_back(max_features);
  }
}

// add a point to the tree (increment count in all cells)
void QuadTree::AddPoint(int row, int col ) {
  cell_feature_count_[0]++;

  // (cell_index, quad_tree_level, row, col )
  AddPointRecursively( 0, 0, row, col );
}

void QuadTree::PrintTree() {
  for (int level = 0; level < num_levels_; ++level) {
    int num_cells_in_this_level = 1 << level;
    for (int  cell_row = 0;  cell_row < num_cells_in_this_level; ++cell_row) {
      for (int cell_col = 0; cell_col < num_cells_in_this_level; ++cell_col) {
        int nIdx = GetCellIndex(cell_row, cell_col, level);
        printf("%d ", cell_feature_count_[nIdx]);
      }
      printf("\n");
    }
    printf("\n");
  }
}

// find all leaves with the minimal count
// pROI [ t l b r ]
void QuadTree::GetMostlyEmptyCellList(std::vector<int>& cells,  int* roi) {
  cells.clear();
  FindMostlyEmptyCellsRecursively(0, cells, roi);
}

void QuadTree::DeactivateCells(std::vector<int>& cells) {
  for( size_t ii=0; ii < cells.size(); ii++ ) {
    active_cells_[cells[ii]] = false;
  }
  DeactivateParentCellsRecursively(0);
}

CellBounds& QuadTree::GetCellBounds(int cell_idx) {
  return cells_bounds_[cell_idx];
}

// add a point to the tree (increment count in all cells)
void QuadTree::ComputeCellBounds() {
  //printf("computing cell bounds: \n");
  cells_bounds_[0] = CellBounds(0, 0, image_height_, image_width_);
  ComputeCellChildrenBoundsRecursively( 0 );
}

void QuadTree::ComputeCellChildrenBoundsRecursively(int cell_idx) {
  int first_child_index = 4*cell_idx + 1;

  // check if the cell is a leaf node
  if ( first_child_index >= num_cells_) {
    // leaf node, return
    return;
  }

  int pt = cells_bounds_[cell_idx].t;
  int pb = cells_bounds_[cell_idx].b;
  int pl = cells_bounds_[cell_idx].l;
  int pr = cells_bounds_[cell_idx].r;
  int w  = (pr - pl)/2;
  int h  = (pb - pt)/2;

  // compute children bounds and
  int kk = first_child_index;
  for (int ii=0; ii<2; ++ii) {
    for (int jj=0; jj<2; ++jj) {
      int t = pt + h * ii;
      int l = pl + w * jj;
      cells_bounds_[kk] = CellBounds(t, l, t+h, l+w);
      ComputeCellChildrenBoundsRecursively(kk);
      kk++;
    }
  }
}

void QuadTree::AddPointRecursively(int cell_idx, int level, int eor, int col) {
  int first_child_index = 4 * cell_idx + 1;

  // check if the cell is a leaf node
  if (first_child_index >= num_cells_) {
    return;
  }

  // Check children
  for (int ii = first_child_index; ii < first_child_index + 4; ++ii) {
    int t = cells_bounds_[ii].t;
    int b = cells_bounds_[ii].b;
    int l = cells_bounds_[ii].l;
    int r = cells_bounds_[ii].r;
    if (eor >= t && eor < b && col >= l && col < r) {
      cell_feature_count_[ii]++;
      /*
        if(cell_feature_count_[ii] >= max_feat_in_cell_per_level_[level+1] ) {
        active_cells_[ii] = false;
        }
      */
      AddPointRecursively(ii, level+1, eor, col);
      return;
    }
  }
}

bool QuadTree::DeactivateParentCellsRecursively(int cell_idx) {

  int first_child_index = 4*cell_idx + 1;

  // leaf return
  if (first_child_index >= num_cells_) {
    return active_cells_[cell_idx];
  }

  // not leaf check if children are all inactive
  bool is_cell_active = false;
  for (int ii=first_child_index; ii < first_child_index + 4; ++ii) {
    bool is_child_active = DeactivateParentCellsRecursively(ii);
    is_cell_active = is_cell_active || is_child_active;
  }

  active_cells_[cell_idx] = is_cell_active;

  return is_cell_active;
}

void QuadTree::FindMostlyEmptyCellsRecursively(
    int cell_idx,
    std::vector<int>& cells,
    int* roi) {

  // check if there is a bounding box
  if (roi) {
    // check if this cell is inside the bounding box
    CellBounds& bounds = cells_bounds_[cell_idx];
    const int t = roi[0];
    const int l = roi[1];
    const int r = roi[2];
    const int b = roi[3];
    if(bounds.b < t || bounds.t > b || bounds.l > r || bounds.r < l ){
      // don't consider this cell
      return;
    }
  }

  int first_child_index = 4*cell_idx + 1;

  // check if the cell is a leaf node
  if (first_child_index >= num_cells_) {
    // active leaf node, add to the vector and return
    if (active_cells_[cell_idx]) {
      cells.push_back(cell_idx);
    }
    return;
  }

  // not a leaf, find children with minimum number of features
  int min_count = INT_MAX;
  for (int ii= first_child_index; ii< first_child_index + 4; ++ii) {
    if (cell_feature_count_[ii] < min_count && active_cells_[ii]) {
      //        if( m_vFeatureCount[ii] < nMinCount ) {
      min_count = cell_feature_count_[ii];
    }
  }

  // if all children are inactive, return
  if( min_count == INT_MAX ) {
    return;
  }

  // search children with minimum number of features
  for (int ii=first_child_index; ii < first_child_index + 4; ++ii) {
    if (cell_feature_count_[ii] == min_count) {
      FindMostlyEmptyCellsRecursively(ii, cells, roi);
    }
  }
}

int QuadTree::GetCellIndex(int row, int col, int level) {
  return GetCellIndexRecursive(0, 0, 0, 0, row, col, level);
}

int QuadTree::GetCellIndexRecursive(int cell_index,
                                    int cell_row,
                                    int cell_col,
                                    int level,
                                    int goal_cell_row,
                                    int goal_cell_col,
                                    int goal_level) {
  int first_child_index = 4*cell_index + 1;
  if (cell_index >= num_cells_) {
    return 0;
  }
  if (cell_row == goal_cell_row &&
      cell_col == goal_cell_col &&
      level == goal_level){
    return cell_index;
  }

  int kk = first_child_index;
  for (int ii=0; ii<2; ++ii) {
    for (int jj=0; jj<2; ++jj) {
      int nRes = GetCellIndexRecursive(kk,
                                       cell_row*2+ii,
                                       cell_col*2+jj,
                                       level+1,
                                       goal_cell_row,
                                       goal_cell_col,
                                       goal_level);
      if (nRes) {
        return nRes;
      }
      kk++;
    }
  }
  return 0; // should never get here.
}
