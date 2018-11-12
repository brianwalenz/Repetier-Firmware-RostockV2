

#ifndef RMATH_H
#define RMATH_H



namespace RMath {
  template<typename X>
  inline
  X
  min(X a, X b) {
    return((a < b) ? a : b);
  }

  template<typename X>
  inline
  X
  max(X a, X b) {
    return((a > b) ? a : b);
  }

  template<typename X>
  inline
  X
  sqr(X a) {
    return(a*a);
  }

  template<typename X>
  inline
  X
  min(X a,X b, X c) {
    if (a < b)
      return a < c ? a : c;
    return b<c ? b : c;
  }
}



#endif  //  RMATH_H
