
int EEPROM_init() {
  const int maxAllowedWrites = 80;
  const int memBase          = 350;
  EEPROM.setMemPool(memBase, EEPROMSizeUno);
  EEPROM.setMaxAllowedWrites(maxAllowedWrites);
  delay(100);
  int addressDouble_kp = EEPROM.getAddress(sizeof(double));
  int addressDouble_ki = EEPROM.getAddress(sizeof(double));
  int addressDouble_kd = EEPROM.getAddress(sizeof(double));

  issuedAdresses(addressDouble_kp, addressDouble_ki, addressDouble_kd);
  return addressDouble_kp;
}

bool EEPROM_Write_double(double _val1, double _val2, double _val3) {
  int addressDouble_kp = EEPROM_init();
  update_PID(addressDouble_kp, _val1, addressDouble_kp+4, _val2, addressDouble_kp+8, _val3);
}

void issuedAdresses(int _addressDouble1, int _addressDouble2, int _addressDouble3) {
  Serial.println(F("-----------------------------------"));
  Serial.println(F("Following adresses have been issued"));
  Serial.println(F("-----------------------------------"));
  Serial.println(F("adress \t\t size"));
  Serial.print(_addressDouble1);    Serial.print(F(" \t\t ")); Serial.print(sizeof(double));  Serial.println(F(" (double)"));
  Serial.print(_addressDouble2);    Serial.print(F(" \t\t ")); Serial.print(sizeof(double));  Serial.println(F(" (double)"));
  Serial.print(_addressDouble3);    Serial.print(F(" \t\t ")); Serial.print(sizeof(double));  Serial.println(F(" (double)"));

}

void update_PID(int _addressDouble_kp, double _kp, int _addressDouble_ki, double _ki, int _addressDouble_kd, double _kd) {
  Serial.println(F("------------------------------"));
  Serial.println(F("updating and retreiving double"));
  Serial.println(F("------------------------------"));

  EEPROM.updateDouble(_addressDouble_kp, _kp);
  EEPROM.updateDouble(_addressDouble_ki, _ki);
  EEPROM.updateDouble(_addressDouble_kd, _kd);

}

void EEPROM_Read_PID() {
  int addressDouble_kp = EEPROM_init();
  double output = 0.0;
  kp = EEPROM.readDouble(addressDouble_kp);
  ki = EEPROM.readDouble(addressDouble_kp+4);
  kd = EEPROM.readDouble(addressDouble_kp+8);

  Serial.print(F("kp: "));
  Serial.print(kp);
  Serial.print(F(" ki: "));
  Serial.print(ki);
  Serial.print(F(" kd: "));
  Serial.print(kd);
  Serial.println(F(""));

}
