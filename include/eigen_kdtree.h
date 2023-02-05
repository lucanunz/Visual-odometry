#include <memory>
#include "split.h"
#include "eigen_covariance.h"
#include "brute_force_search.h"

template <typename IteratorType_>
class TreeNode_ {
public:
  using IteratorType = IteratorType_;
  using PointType = typename IteratorType_::value_type;
  using Scalar    = typename PointType::Scalar;
  static constexpr int Dim = PointType::RowsAtCompileTime;
  using CovarianceType = Eigen::Matrix<Scalar, Dim-1, Dim-1>;
  using ThisType  = TreeNode_<IteratorType>;
  using PtrType = std::unique_ptr < ThisType  >;
  using AnswerType = std::vector<PointType* >;
  
  TreeNode_(IteratorType begin_,
            IteratorType end_,
            int max_points_in_leaf=20):
    _begin(begin_),
    _end(end_)
  {
    int num_points=std::distance(_begin, _end);
    if (num_points < max_points_in_leaf)
      return;
    CovarianceType cov;
    computeMeanAndCovariance(_mean, cov, _begin, _end);
    _normal = largestEigenVector(cov);

    IteratorType middle = split(_begin, _end,
                                [&](const PointType& p)->bool {
                                  return (p.tail(Dim-1)-_mean).dot(_normal) < Scalar(0);
                                }
                                );
    _left  = PtrType(new ThisType(_begin, middle, max_points_in_leaf) );
    _right = PtrType(new ThisType(middle, _end,   max_points_in_leaf) );
  }

  void fastSearch(AnswerType& answers,
                  const PointType& query,
                  Scalar norm) {
    if (! _left && !_right) {
      bruteForceSearch(answers, _begin, _end, query, norm);
      return;
    }
    Scalar distance_from_split_plane =  (query.tail(Dim-1)-_mean).dot(_normal);
    if (distance_from_split_plane<Scalar(0))
      _left->fastSearch(answers,query,norm);
    else
      _right->fastSearch(answers,query,norm);
  }

  void fullSearch(AnswerType& answers,
                  const PointType& query,
                  Scalar norm) {
    if (! _left && !_right) {
      bruteForceSearch(answers, _begin, _end, query, norm);
      return;
    }
    Scalar distance_from_split_plane =  (query.tail(Dim-1)-_mean).dot(_normal);
    if (distance_from_split_plane < -norm )
      _left->fullSearch(answers,query, norm);
    else if (distance_from_split_plane > norm )
      _right->fullSearch(answers,query,norm);
    else {
     _left->fullSearch(answers,query, norm);
      _right->fullSearch(answers,query,norm);
    }
  }

  // this returns the closest point in the set,
  // among those whose distance from query is smaller than norm
  // doing an approximate search
  PointType* bestMatchFast(const PointType& query,
                           Scalar norm) {
    if (! _left && !_right) {
      return bruteForceBestMatch(_begin, _end, query, norm);
    }
    Scalar distance_from_split_plane =  (query.tail(Dim-1)-_mean).dot(_normal);
    if (distance_from_split_plane<Scalar(0))
      return _left->bestMatchFast(query, norm);
    else
      return _right->bestMatchFast(query, norm);
  }

  // this returns the closest point in the set,
  // among those whose distance from query is smaller than norm
  // doing a full search
  PointType* bestMatchFull(const PointType& query,
                           Scalar norm) {
    if (! _left && !_right) {
      return bruteForceBestMatch(_begin, _end, query, norm);
    }
    Scalar distance_from_split_plane =  (query.tail(Dim-1)-_mean).dot(_normal);

    if (distance_from_split_plane < -norm )
      return _left->bestMatchFull(query, norm);

    if (distance_from_split_plane > norm )
      return _right->bestMatchFull(query, norm);

    PointType* p_left  = _left->bestMatchFull(query, norm);
    PointType* p_right = _right->bestMatchFull(query, norm);
    Scalar d_left=norm*norm;
    Scalar d_right=norm*norm;
    if (p_left)
      d_left=((*p_left)-query).tail(Dim-1).squaredNorm();
    if (p_right)
      d_right=((*p_right)-query).tail(Dim-1).squaredNorm();
    if (d_left<d_right)
      return p_left;

    return p_right;
  }

protected:
  Eigen::Matrix<Scalar,Dim-1, 1> _mean, _normal;
  IteratorType _begin, _end;
  PtrType _left, _right;
};
