/*!
 * @file DFRobot_LCD.cpp
 * @brief DFRobot's LCD
 * @n High Accuracy Ambient Light Sensor
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [yangyang](971326313@qq.com)
 * @version  V1.0
 * @date  2017-2-10
 */


#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "DFRobot_LCD.h"






const uint8_t color_define[4][3] = 
{
    {255, 255, 255},            // white
    {255, 0, 0},                // red
    {0, 255, 0},                // green
    {0, 0, 255},                // blue
};

/*******************************public*******************************/
DFRobot_LCD::DFRobot_LCD(uint8_t lcd_cols,uint8_t lcd_rows,uint8_t lcd_Addr,uint8_t RGB_Addr)
{
  _lcdAddr = lcd_Addr;
  _RGBAddr = RGB_Addr;
  _cols = lcd_cols;
  _rows = lcd_rows;
}

void DFRobot_LCD::init()
{
	_showfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	begin(_cols, _rows);
}

void DFRobot_LCD::clear()
{
    command(LCD_CLEARDISPLAY);        // clear display, set cursor position to zero
    vTaskDelay(pdMS_TO_TICKS(2));          // this command takes a long time!
}

void DFRobot_LCD::home()
{
    command(LCD_RETURNHOME);        // set cursor position to zero
    vTaskDelay(pdMS_TO_TICKS(2));        // this command takes a long time!
}

void DFRobot_LCD::noDisplay()
{
    _showcontrol &= ~LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | _showcontrol);
}

void DFRobot_LCD::display() {
    _showcontrol |= LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | _showcontrol);
}

void DFRobot_LCD::stopBlink()
{
    _showcontrol &= ~LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | _showcontrol);
}
void DFRobot_LCD::blink()
{
    _showcontrol |= LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | _showcontrol);
}

void DFRobot_LCD::noCursor()
{
    _showcontrol &= ~LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | _showcontrol);
}

void DFRobot_LCD::cursor() {
    _showcontrol |= LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | _showcontrol);
}

void DFRobot_LCD::scrollDisplayLeft(void)
{
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void DFRobot_LCD::scrollDisplayRight(void)
{
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void DFRobot_LCD::leftToRight(void)
{
    _showmode |= LCD_ENTRYLEFT;
    command(LCD_ENTRYMODESET | _showmode);
}

void DFRobot_LCD::rightToLeft(void)
{
    _showmode &= ~LCD_ENTRYLEFT;
    command(LCD_ENTRYMODESET | _showmode);
}

void DFRobot_LCD::noAutoscroll(void)
{
    _showmode &= ~LCD_ENTRYSHIFTINCREMENT;
    command(LCD_ENTRYMODESET | _showmode);
}

void DFRobot_LCD::autoscroll(void)
{
    _showmode |= LCD_ENTRYSHIFTINCREMENT;
    command(LCD_ENTRYMODESET | _showmode);
}

void DFRobot_LCD::customSymbol(uint8_t location, uint8_t charmap[])
{

    location &= 0x7; // we only have 8 locations 0-7
    command(LCD_SETCGRAMADDR | (location << 3));
    
    
    uint8_t data[9];
    data[0] = 0x40;
    for(int i=0; i<8; i++)
    {
        data[i+1] = charmap[i];
    }
    send(data, 9);
}

void DFRobot_LCD::setCursor(uint8_t col, uint8_t row)
{

    col = (row == 0 ? col|0x80 : col|0xc0);
    uint8_t data[3] = {0x80, col};

    send(data, 2);

}

void DFRobot_LCD::setRGB(uint8_t r, uint8_t g, uint8_t b)
{
    setReg(REG_RED, r);
    setReg(REG_GREEN, g);
    setReg(REG_BLUE, b);
}


void DFRobot_LCD::setColor(uint8_t color)
{
    if(color > 3)return ;
    setRGB(color_define[color][0], color_define[color][1], color_define[color][2]);
}

void DFRobot_LCD::blinkLED(void)
{
    ///< blink period in seconds = (<reg 7> + 1) / 24
    ///< on/off ratio = <reg 6> / 256
    setReg(0x07, 0x17);  // blink every second
    setReg(0x06, 0x7f);  // half on, half off
}

void DFRobot_LCD::noBlinkLED(void)
{
    setReg(0x07, 0x00);
    setReg(0x06, 0xff);
}

inline size_t DFRobot_LCD::write(uint8_t value)
{

    uint8_t data[3] = {0x40, value};
    send(data, 2);
    return 1; // assume sucess
}

inline void DFRobot_LCD::command(uint8_t value)
{
    uint8_t data[3] = {0x80, value};
    send(data, 2);
}

void DFRobot_LCD::blink_on(){
	blink();
}

void DFRobot_LCD::blink_off(){
	stopBlink();
}

void DFRobot_LCD::cursor_on(){
	cursor();
}

void DFRobot_LCD::cursor_off(){
	noCursor();
}

void DFRobot_LCD::setBacklight(uint8_t new_val){
	if(new_val){
		blinkLED();		// turn backlight on
	}else{
		noBlinkLED();		// turn backlight off
	}
}

void DFRobot_LCD::load_custom_character(uint8_t char_num, uint8_t *rows){
		customSymbol(char_num, rows);
}

void DFRobot_LCD::printstr(const char c[]){
	///< This function is not identical to the function used for "real" I2C displays
	///< it's here so the user sketch doesn't have to be changed 
	//print(c);
    printf("Oops! DFRobot_LCD::printstr is not written yet!\n");
}

/*******************************private*******************************/
void DFRobot_LCD::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) 
{
    if (lines > 1) {
        _showfunction |= LCD_2LINE;
    }
    _numlines = lines;
    _currline = 0;

    ///< for some 1 line displays you can select a 10 pixel high font
    if ((dotsize != 0) && (lines == 1)) {
        _showfunction |= LCD_5x10DOTS;
    }

    ///< SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
    ///< according to datasheet, we need at least 40ms after power rises above 2.7V
    ///< before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
    vTaskDelay(pdMS_TO_TICKS(50));


    ///< this is according to the hitachi HD44780 datasheet
    ///< page 45 figure 23

    ///< Send function set command sequence
    command(LCD_FUNCTIONSET | _showfunction);
    vTaskDelay(pdMS_TO_TICKS(5));  // wait more than 4.1ms
	
	///< second try
    command(LCD_FUNCTIONSET | _showfunction);
    vTaskDelay(pdMS_TO_TICKS(5));

    ///< third go
    command(LCD_FUNCTIONSET | _showfunction);




    ///< turn the display on with no cursor or blinking default
    _showcontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    display();

    ///< clear it off
    clear();

    ///< Initialize to default text direction (for romance languages)
    _showmode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    ///< set the entry mode
    command(LCD_ENTRYMODESET | _showmode);
    
    
    ///< backlight init
    setReg(REG_MODE1, 0);
    ///< set LEDs controllable by both PWM and GRPPWM registers
    setReg(REG_OUTPUT, 0xFF);
    ///< set MODE2 values
    ///< 0010 0000 -> 0x20  (DMBLNK to 1, ie blinky mode)
    setReg(REG_MODE2, 0x20);
    
    setColorWhite();

}

void DFRobot_LCD::send(uint8_t *data, uint8_t len)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);        // transmit to device #4
     i2c_master_write_byte(cmd, (_lcdAddr << 1), ACK_CHECK_EN);
    //i2c_master_write_byte(cmd, _lcdAddr, ACK_CHECK_EN);
    for(int i=0; i<len; i++) {
        i2c_master_write_byte(cmd, data[i], ACK_CHECK_EN);
		vTaskDelay(pdMS_TO_TICKS(5));
    }
    i2c_master_stop(cmd);                     // stop transmitting

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // do not progress if data is not OK
    if (ret == ESP_OK) {
    } else {
        ESP_LOGE(TAG, "Failed to SEND data over link!");
        //printf("%s\n",esp_err_to_name(ret));
    }



}

void DFRobot_LCD::setReg(uint8_t addr, uint8_t data)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd); // transmit to device #4
    //i2c_master_write_byte(cmd, (addr << 1), ACK_CHECK_EN);
    //i2c_master_write_byte(cmd, (_RGBAddr<<1), ACK_CHECK_EN);
    
    // RGB address for V2.0 LCD screen
    i2c_master_write_byte(cmd, 0x2D, ACK_CHECK_EN);
    vTaskDelay(pdMS_TO_TICKS(5));
    i2c_master_write_byte(cmd, 0x80, ACK_CHECK_EN);
    vTaskDelay(pdMS_TO_TICKS(5));
    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);
    vTaskDelay(pdMS_TO_TICKS(5));

    i2c_master_write_byte(cmd, 0x80, ACK_CHECK_EN);
    vTaskDelay(pdMS_TO_TICKS(5));
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);

    i2c_master_stop(cmd);    // stop transmitting

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // do not progress if data is not OK
    if (ret == ESP_OK) {
    } else {
        ESP_LOGE(TAG, "Failed to SET REG!");
    }

}

/************************unsupported API functions***************************/
void DFRobot_LCD::off(){}
void DFRobot_LCD::on(){}
void DFRobot_LCD::setDelay (int cmdDelay,int charDelay) {}
uint8_t DFRobot_LCD::status(){return 0;}
uint8_t DFRobot_LCD::keypad (){return 0;}
uint8_t DFRobot_LCD::init_bargraph(uint8_t graphtype){return 0;}
void DFRobot_LCD::draw_horizontal_graph(uint8_t row, uint8_t column, uint8_t len,  uint8_t pixel_col_end){}
void DFRobot_LCD::draw_vertical_graph(uint8_t row, uint8_t column, uint8_t len,  uint8_t pixel_row_end){}
void DFRobot_LCD::setContrast(uint8_t new_val){}
