#ifndef _OPTIMIZER_INIT_H_
#define _OPTIMIZER_INIT_H_
#include<g2o/core/sparse_optimizer.h>
#include<g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include<Eigen/Core>

//最基本的对g2o优化器的初始化，有别的选项可以再在这里集成
void initG2oOptimizer(g2o::SparseOptimizer& optimizer);

#endif
