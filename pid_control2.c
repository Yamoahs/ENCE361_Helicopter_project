

int altPIDControl(int hgt_percent,double dt)
{
  static signed long altError;
  static signed long altProportion;
  static signed long altIntergral;
  static signed long altDerivative;
  static signed long altPrevError;
  int control;

  float altKp;
  float altKi;
  float altKd;

  altError = desiredHeight - hgt_percent;

  altProportion = altError * altKp;
  altIntergral += error * dt;
  altDerivative = (altError - altPrevError) / dt;

  altPrevError = altError;

  control = altProportion + altIntergral * altKi + altDerivative * altKd;

  return control;
}

int yawPIDControl(double dt)
{
  static signed long yawError;
  static signed long yawProportion;
  static signed long yawIntergral;
  static signed long yawDerivative;
  static signed long yawPrevError;
  int control;

  float yawKp;
  float yawKi;
  float yawKd;

  yawError = desiredHeight - hgt_percent;

  yawProportion = yawError * yawKp;
  yawIntergral += error * dt;
  yawDerivative = (yawError - yawPrevError) / dt;

  yawPrevError = yawError;

  control = yawProportion + yawIntergral * yawKi + yawDerivative * yawKd;

  return control;
}
