#ifndef TOOLS_H_
#define TOOLS_H_
#include <time.h>
#include <fstream>

using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  ofstream f_log;

  void Log(double px,double py,double psi,double v,double cte,double epsi,double steer,double throttle);
};

#endif /* TOOLS_H_ */
