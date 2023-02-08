#pragma once

// split functions
// takes two iterators (begin and end)
// and reorders the data so that the items for which the predicate
// is true are at the beginning, tho others at the end
// it returns the iterator to the splitting element between the two classes
template <typename IteratorType_, typename PredicateType_>
IteratorType_ split(IteratorType_ begin,
                    IteratorType_ end,
                    PredicateType_ predicate) {
  using ValueType    = typename IteratorType_::value_type;
  auto lower=begin;  //first
  // upper is an iterator at the end, it is a reverse iterator,
  // and moves backward when incremented;
  // we apply the predicate to each point,
  // and if the result is positive
  //   we leave the point where it is
  // otherwise
  //   we move the point in the other extrema, and increment the extrema
  // the iterations end when the upper and lower ends match
  auto upper=std::make_reverse_iterator(end); 
  while (lower!=upper.base()) {
    ValueType& v_lower=*lower;
    ValueType& v_upper=*upper;
    if ( predicate(v_lower) ){
      ++lower;
    } else {
      std::swap(v_lower,v_upper);
      ++upper;
    }
  }
  return upper.base();
}
