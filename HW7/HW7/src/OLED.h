/*
 * File:   OLED.h
 * Author: Shitij
 *
 * Created on 16 April, 2015, 10:26 PM
 */

void oled_pos_set(int row,int col);
int oled_row_get();
int oled_col_get();
void print_string_oled(char *str);
void screen_on();

