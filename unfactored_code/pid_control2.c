

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

  altKp = 0.5;
  altKi = 0.0009;
  altKd = 0.8;

  altError = desiredHeight - hgt_percent;

  altProportion = altError * altKp;
  altIntergral += error * dt;
  altDerivative = (altError - altPrevError) / dt;

  altPrevError = altError;

  control = altProportion + altIntergral * altKi + altDerivative * altKd;

  if(main_duty > 98) main_duty = 98;
  if(main_duty < 2) main_duty = 2;

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

  yawKp = 0.6;
  yawKi = 0.009;
  yawKd = 2.5;

  yawError = desiredHeight - hgt_percent;

  yawProportion = yawError * yawKp;
  yawIntergral += error * dt;
  yawDerivative = (yawError - yawPrevError) / dt;

  yawPrevError = yawError;

  control = yawProportion + yawIntergral * yawKi + yawDerivative * yawKd;

  if(tail_duty > 98) tail_duty = 98;
  if(tail_duty < 2) tail_duty = 2;

  return control;
}
