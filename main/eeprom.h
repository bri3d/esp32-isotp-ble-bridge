#ifndef EEPROM_H
#define EEPROM_H

esp_err_t 	eeprom_init();
void		eeprom_deinit();
esp_err_t	eeprom_commit();
void 		eeprom_defaults();
esp_err_t 	eeprom_write_str(char* address, char* data);
char*		eeprom_read_str(char* address);
esp_err_t 	eeprom_write_int(char* address, int32_t data);
int32_t		eeprom_read_int(char* address);

#endif