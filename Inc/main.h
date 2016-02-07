#ifndef _MAIN_H_
#define _MAIN_H_

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
void delay_ms(unsigned long num_ms);
void get_ms(unsigned long *count);
int reg_int_cb(struct int_param_s *int_param);
int min(int a, int b);
void __no_operation();

#endif
