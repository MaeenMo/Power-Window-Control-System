// lcd_i2c.c:
#include "lcd_i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"

// timing helpers
static void delay_us(uint32_t us) {
    // crude delay: adjust as needed
    volatile uint32_t cnt = (SysCtlClockGet()/3000000) * us;
    while(cnt--) { __asm("nop"); }
}

// low-level write to expander
static void writeExp(uint8_t b) {
    I2CMasterSlaveAddrSet(I2C0_BASE, LCD_I2C_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, b);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C0_BASE));
}

// strobe the “enable” line (E bit = 0x04)
static void strobe(uint8_t data) {
    writeExp(data | 0x04);
    delay_us(1);
    writeExp(data & ~0x04);
    delay_us(50);
}

// send either a command (rs=0) or data (rs=1) nibble-wise
static void sendByte(uint8_t val, bool rs) {
    uint8_t high = (val & 0xF0)        | (rs?0x01:0x00) | 0x08; // backlight=1
    uint8_t low  = ((val<<4) & 0xF0)   | (rs?0x01:0x00) | 0x08;
    writeExp(high);
    strobe(high);
    writeExp(low);
    strobe(low);
}

bool LCD_I2C_Init(void) {
    // assume I2C0 already init’d at 100 kHz
    // 4-bit init sequence:
    sendByte(0x33, false);
    sendByte(0x32, false);
    sendByte(0x28, false); // 2-line, 5×8
    sendByte(0x0C, false); // display on, cursor off
    sendByte(0x06, false); // entry mode
    sendByte(0x01, false); // clear
    SysCtlDelay(SysCtlClockGet()/ (3*100)); // ~5 ms
    return true;
}

void LCD_I2C_CreateChar(uint8_t slot, const uint8_t data[8]) {
    // slot 0..7
    sendByte(0x40 | ((slot&7)<<3), false);
    for(int i=0;i<8;i++) sendByte(data[i], true);
}

void LCD_I2C_Clear(void) {
    sendByte(0x01, false);
    SysCtlDelay(SysCtlClockGet()/ (3*100)); // ~5 ms
}

void LCD_I2C_SetCursor(uint8_t row, uint8_t col) {
    static const uint8_t offsets[] = {0x00,0x40,0x14,0x54};
    sendByte(0x80 | (offsets[row&1] + (col&0x0F)), false);
}

void LCD_I2C_Print(const char *s) {
    while(*s) {
        sendByte((uint8_t)*s++, true);
    }
}

void LCD_I2C_WriteChar(uint8_t c) {
    sendByte(c, true);
}