
float DEMA_filter(float _angle) {
  if (_angle != 0.00 and abs(_angle) <= 360) {    // fillter the obvious

    ema = EMA_function(alpha, _angle, ema);
    ema_ema = EMA_function(alpha, ema, ema_ema);

    float DEMA = 2 * ema - ema_ema;
    return DEMA;
  }
}

float US_DEMA_filter(float _data) {
  ema_US = EMA_function_US(alpha_US, _data, ema_US);
  ema_ema_US = EMA_function_US(alpha_US, ema_US, ema_ema_US);

  float DEMA_US = 2 * ema_US - ema_ema_US;
  return DEMA_US;
}

float EMA_function_US(float alpha_US, float latest_US, float stored_US) {
  return alpha_US * latest_US + (1 - alpha_US) * stored_US;
}

float EMA_function(float alpha, float latest, float stored) {
  return alpha * latest + (1 - alpha) * stored;
}
