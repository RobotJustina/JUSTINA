//polynome coef after octave polyfit to TORSO height
double a = -0.000000000050838;
double b = 0.000000077108;
double c = -0.000046758;
double d = 0.014476;
double e = -2.4140;
double f = 203.87;

double sharp2cm (int raw_value)
{
  return  a * pow(raw_value, 5) + b * pow(raw_value, 4) + c * pow(raw_value, 3) + d * pow(raw_value, 2) + e * raw_value + f;
}

void move_motor_up()
{
  motor_tronco.write(MOTOR_UP);
}

void move_motor_down()
{
  motor_tronco.write(MOTOR_DOWN);
}

void stop_motor()
{
  motor_tronco.write(MOTOR_STOP);  
}

